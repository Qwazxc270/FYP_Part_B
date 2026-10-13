#!/usr/bin/env python3
"""Supervisor node (Stages 2-4), wired to the team's FYP_Part_B stack.

All platform-specific topic names live in config/platforms/<platform>.yaml
(selected with the ``platform`` launch arg). Message *types* consumed are
the team's existing ones; conversion happens in fsae_supervisor.team_adapters.

Inputs (gazebo profile):
    /imu                               sensor_msgs/Imu
    /scan                              sensor_msgs/LaserScan
    /wheelrl_motor/velocity, /wheelrr_motor/velocity   std_msgs/Float32 [rad/s]
    /qcar/base_f{l,r}_controller/command               std_msgs/Float64 [rad]
    /odometry/local  (odom_in)         nav_msgs/Odometry  nav_filter_node: r1, r3, (future r4)
    /odometry/filtered (odom_ref_in)   nav_msgs/Odometry  ekf_slam_node: r13 filter_gap only
    /cone_detections_camera            qcar_navigation/ConeDetectionArray
    /cone_detections_lidar             geometry_msgs/PoseArray
    /qcar/trajectory_topic             qcar_guidance/TrajectoryMessage (v_ref)
  optional (empty topic = not subscribed; see INTEGRATION_TODO.md):
    ekf_state_in  fsae_supervisor/EkfState  (NIS)
    mpc_stats_in  fsae_supervisor/MpcStats  (cte, solve time, v_ref)
    encoder_speed_in std_msgs/Float32 [m/s] (hardware single encoder)
    steer_meas_in std_msgs/Float64 [rad]    (measured steering)

Outputs:
    /supervisor/residuals   ResidualVector
    /supervisor/posterior   ModePosterior
    /supervisor/action      SupervisorAction
    /supervisor/fault_flags FaultFlags
"""

from __future__ import annotations

import threading
from dataclasses import dataclass, field, replace
from typing import Optional

import numpy as np
import rospy

from std_msgs.msg      import Header, Float32, Float64
from std_srvs.srv      import Trigger, TriggerResponse
from sensor_msgs.msg   import Imu, LaserScan
from nav_msgs.msg      import Odometry
from geometry_msgs.msg import PoseArray

from fsae_supervisor import (
    SupervisorConfig, ResidualGenerator, IMM, ActionPolicy,
    SupervisorInputs, IMUData, EncoderData, ConeObs, LidarScan, EKFSnapshot,
    FaultFlagInputs,
)
from fsae_supervisor import team_adapters as ta
from fsae_supervisor.msg import (
    ResidualVector, ModePosterior, SupervisorAction as SupervisorActionMsg,
    FaultFlags, MpcStats, EkfState,
)

# Team message types — imported lazily so the node still starts (with
# that input disabled) if the team package isn't built in this workspace.
try:
    from qcar_navigation.msg import ConeDetectionArray
except ImportError:          # pragma: no cover
    ConeDetectionArray = None
try:
    from qcar_guidance.msg import TrajectoryMessage
except ImportError:          # pragma: no cover
    TrajectoryMessage = None

SENSOR_STALE_S = 0.5     # IMU / encoder considered invalid after this


# =====================================================================
@dataclass
class Caches:
    imu_w:        Optional[float] = None
    imu_rx:       Optional[float] = None          # receipt time [s]
    w_rl:         Optional[float] = None
    w_rr:         Optional[float] = None
    enc_speed:    Optional[float] = None          # direct m/s (hardware)
    enc_rx:       Optional[float] = None
    d_fl:         Optional[float] = None
    d_fr:         Optional[float] = None
    steer_meas:   Optional[float] = None
    cam_range:    Optional[np.ndarray] = None     # cleared every tick
    lid_range:    Optional[np.ndarray] = None     # cleared every tick
    scan_ranges:  Optional[np.ndarray] = None
    ekf_x:        Optional[np.ndarray] = None
    ekf_P:        Optional[np.ndarray] = None
    ekf_last_nis: Optional[float] = None
    ekf_updated:  bool = False
    traj_v:       Optional[float] = None
    mpc_seen:     bool = False
    mpc_time_ms:  float = 0.0
    mpc_budget_ms: float = 50.0
    mpc_delta:    Optional[float] = None
    mpc_v_ref:    Optional[float] = None
    cte:          float = 0.0
    throttle_cmd: Optional[float] = None


# =====================================================================
class SupervisorNode:
    def __init__(self) -> None:
        rospy.init_node("supervisor_node")

        ns = rospy.get_param("~config_ns", "supervisor")
        self.cfg = SupervisorConfig.from_rosparam(ns)
        pl = self.cfg.platform
        off = [n for n, e in zip(self.cfg.residual.names, self.cfg.channels_enabled) if not e]
        rospy.loginfo("supervisor_node: platform=%s  %.1f Hz  %d residuals (%d disabled: %s)  %d modes",
                      pl.name, self.cfg.rate_hz, self.cfg.residual.n, len(off),
                      ", ".join(off) or "none", self.cfg.mode.n_modes)

        self.resid  = ResidualGenerator(self.cfg)
        self.imm    = IMM(self.cfg)
        self.policy = ActionPolicy(self.cfg)

        self._lock  = threading.Lock()
        self._cache = Caches()
        self._names_published = False

        t = self.cfg.topics
        self.pub_res   = rospy.Publisher(t.residuals_out,   ResidualVector,      queue_size=10)
        self.pub_post  = rospy.Publisher(t.posterior_out,   ModePosterior,       queue_size=10)
        self.pub_act   = rospy.Publisher(t.action_out,      SupervisorActionMsg, queue_size=10)
        self.pub_flags = rospy.Publisher(t.fault_flags_out, FaultFlags,          queue_size=10)

        self._sub(t.imu_in,          Imu,       self._cb_imu,   50)
        self._sub(t.wheel_rl_vel_in, Float32,   self._cb_w_rl,  50)
        self._sub(t.wheel_rr_vel_in, Float32,   self._cb_w_rr,  50)
        self._sub(t.encoder_speed_in, Float32,  self._cb_enc_speed, 50)
        self._sub(t.steer_fl_cmd_in, Float64,   self._cb_d_fl,  20)
        self._sub(t.steer_fr_cmd_in, Float64,   self._cb_d_fr,  20)
        self._sub(t.steer_meas_in,   Float64,   self._cb_steer_meas, 20)
        self._sub(t.odom_in,         Odometry,  self._cb_odom,  20)
        fg = self.cfg.filter_gap
        self._hist_local = ta.PoseHistory(fg.history_s)
        self._hist_ref   = ta.PoseHistory(fg.history_s)
        self._fg_on = self.cfg.channel_enabled("filter_gap") and bool(t.odom_ref_in)
        if self._fg_on:
            self._sub(t.odom_ref_in, Odometry, self._cb_odom_ref, 20)
        self._sub(t.cones_lidar_in,  PoseArray, self._cb_lid_cones, 20)
        self._sub(t.lidar_scan_in,   LaserScan, self._cb_scan,  5)
        self._sub(t.ekf_state_in,    EkfState,  self._cb_ekf,   20)
        self._sub(t.mpc_stats_in,    MpcStats,  self._cb_mpc,   20)
        if ConeDetectionArray is not None:
            self._sub(t.cones_cam_in, ConeDetectionArray, self._cb_cam_cones, 20)
        elif t.cones_cam_in:
            rospy.logwarn("supervisor_node: qcar_navigation msgs not found; camera cones disabled")
        if TrajectoryMessage is not None:
            self._sub(t.trajectory_in, TrajectoryMessage, self._cb_traj, 5)
        elif t.trajectory_in:
            rospy.logwarn("supervisor_node: qcar_guidance msgs not found; v_ref unavailable")

        # Start-up gate (integration fix): the pipeline is not stepped until
        # every *required* input has delivered at least one message.
        # Without this, cam/lid freshness timers run from node start and
        # the cones_both_timeout override latches Critical before Gazebo
        # has published anything (observed in the smoke test).
        req = list(rospy.get_param("~required_inputs",
                              ["imu_in", "odom_in", "cones_cam_in", "cones_lidar_in", "lidar_scan_in"]))
        if self._fg_on and "odom_ref_in" not in req:
            req.append("odom_ref_in")      # filter_gap needs the SLAM estimate too
        self._waiting = {k for k in req if getattr(t, k)}
        self._armed = not self._waiting
        rospy.Service("~reset", Trigger, self._on_reset)

        self._timer = rospy.Timer(rospy.Duration(self.cfg.dt), self._on_tick)

    def _sub(self, topic: str, typ, cb, q: int) -> None:
        if topic:
            key = next((k for k, v in self.cfg.topics.as_dict().items() if v == topic), topic)

            def _wrapped(msg, _cb=cb, _key=key):
                if not self._armed and _key in self._waiting:
                    self._waiting.discard(_key)
                    if not self._waiting:
                        self._armed = True
                        rospy.loginfo("supervisor_node: all required inputs seen — supervision ARMED")
                _cb(msg)
            rospy.Subscriber(topic, typ, _wrapped, queue_size=q)
            rospy.loginfo("supervisor_node:   sub %-40s [%s]", topic, typ._type)

    def _on_reset(self, _req) -> TriggerResponse:
        """Clear latched Critical / hysteresis / IMM posterior. Intended to be
        called together with /mission_manager/set_mission reset."""
        with self._lock:
            self.resid.reset(); self.imm.reset(); self.policy.reset()
            self._hist_local.clear(); self._hist_ref.clear()
        rospy.logwarn("supervisor_node: RESET (latch, hysteresis, posterior cleared)")
        return TriggerResponse(success=True, message="supervisor reset")

    @staticmethod
    def _now() -> float:
        return rospy.get_time()

    # ------------------------------------------------------------------
    #  Callbacks — O(1), cache only
    # ------------------------------------------------------------------
    def _cb_imu(self, m: Imu) -> None:
        with self._lock:
            self._cache.imu_w = float(m.angular_velocity.z)
            self._cache.imu_rx = self._now()

    def _cb_w_rl(self, m: Float32) -> None:
        with self._lock:
            self._cache.w_rl = float(m.data); self._cache.enc_rx = self._now()

    def _cb_w_rr(self, m: Float32) -> None:
        with self._lock:
            self._cache.w_rr = float(m.data); self._cache.enc_rx = self._now()

    def _cb_enc_speed(self, m: Float32) -> None:
        with self._lock:
            self._cache.enc_speed = float(m.data); self._cache.enc_rx = self._now()

    def _cb_d_fl(self, m: Float64) -> None:
        with self._lock:
            self._cache.d_fl = float(m.data)

    def _cb_d_fr(self, m: Float64) -> None:
        with self._lock:
            self._cache.d_fr = float(m.data)

    def _cb_steer_meas(self, m: Float64) -> None:
        with self._lock:
            self._cache.steer_meas = float(m.data)

    def _cb_odom(self, m: Odometry) -> None:
        q = m.pose.pose.orientation
        yaw = ta.yaw_from_quaternion(q.x, q.y, q.z, q.w)
        x, P = ta.odometry_to_ekf(m.pose.pose.position.x, m.pose.pose.position.y,
                                  yaw, m.twist.twist.linear.x, m.pose.covariance)
        stamp = self._stamp(m)
        with self._lock:
            self._cache.ekf_x = x
            self._cache.ekf_P = P
            self._hist_local.push(stamp, x[0], x[1], x[2])

    def _cb_odom_ref(self, m: Odometry) -> None:
        q = m.pose.pose.orientation
        yaw = ta.yaw_from_quaternion(q.x, q.y, q.z, q.w)
        stamp = self._stamp(m)
        with self._lock:
            self._hist_ref.push(stamp, m.pose.pose.position.x, m.pose.pose.position.y, yaw)

    def _stamp(self, m) -> float:
        """Header stamp (for time alignment); receipt time if unset."""
        st = m.header.stamp.to_sec()
        return st if st > 0 else self._now()

    def _cb_cam_cones(self, m) -> None:
        d = m.detections
        r = ta.cone_ranges_from_detections([c.range for c in d],
                                           [c.position.x for c in d],
                                           [c.position.y for c in d])
        with self._lock:
            self._cache.cam_range = r

    def _cb_lid_cones(self, m: PoseArray) -> None:
        r = ta.cone_ranges_from_points([p.position.x for p in m.poses],
                                       [p.position.y for p in m.poses])
        with self._lock:
            self._cache.lid_range = r

    def _cb_scan(self, m: LaserScan) -> None:
        rmin = max(float(m.range_min), self.cfg.platform.scan_range_min)
        arr = ta.finite_scan_ranges(m.ranges, rmin)
        with self._lock:
            self._cache.scan_ranges = arr

    def _cb_traj(self, m) -> None:
        with self._lock:
            self._cache.traj_v = float(m.velocity)

    def _cb_ekf(self, m: EkfState) -> None:
        # Optional richer EKF message (TODO for teammates): only NIS is used
        # from it; pose/cov still come from Odometry.
        with self._lock:
            self._cache.ekf_last_nis = float(m.last_nis) if np.isfinite(m.last_nis) else None
            self._cache.ekf_updated = bool(m.updated_this_step)

    def _cb_mpc(self, m: MpcStats) -> None:
        with self._lock:
            c = self._cache
            c.mpc_seen = True
            c.mpc_time_ms = float(m.compute_time_ms)
            if m.budget_ms > 0:
                c.mpc_budget_ms = float(m.budget_ms)
            c.mpc_delta = float(m.delta_cmd)
            c.mpc_v_ref = float(m.v_ref)
            c.cte = float(m.cte)

    # ------------------------------------------------------------------
    def _on_tick(self, _event) -> None:
        if not self._armed:
            self._publish_waiting()
            return
        with self._lock:
            snap = replace(self._cache)
            # Cones are on-event: clear so freshness timers grow on dropout.
            self._cache.cam_range = None
            self._cache.lid_range = None
            self._cache.ekf_last_nis = None

        inp = self._build_inputs(snap, self._now())
        r, r_valid, flags = self.resid.step(inp, self.cfg.dt)
        self.imm.step(r, r_valid)
        mu_fine   = self.imm.mu
        mu_coarse = self.imm.coarse()

        act = self.policy.step(mu_fine, FaultFlagInputs(
            cones_both_timeout=flags.cones_both_timeout,
            steering_fault=flags.steering_fault,
            drive_fault=flags.drive_fault,
            obstacle_detected=flags.obstacle_detected,
        ))

        now = rospy.Time.now()
        self._publish_residuals(now, r, r_valid)
        self._publish_posterior(now, mu_fine, mu_coarse)
        self._publish_flags(now, flags)
        self._publish_action(now, act, mu_fine, mu_coarse)

    def _publish_waiting(self) -> None:
        """Before arming: speed_scale 0 (don't drive), estop False (don't
        latch the mission FSM), mode_name 'Initialising'."""
        msg = SupervisorActionMsg()
        msg.header = self._make_header(rospy.Time.now())
        msg.mode_coarse = 1
        msg.mode_fine = 1
        msg.mode_name = "Initialising"
        msg.mode_name_fine = "Initialising"
        msg.speed_scale = 0.0
        msg.estop = False
        msg.fault_isolation = "waiting_for: " + ", ".join(sorted(self._waiting))
        msg.mu_fine = list(self.imm.mu)
        msg.mu_coarse = list(self.imm.coarse())
        self.pub_act.publish(msg)
        rospy.loginfo_throttle(5, "supervisor_node: waiting for inputs: %s" % ", ".join(sorted(self._waiting)))

    def _build_inputs(self, c: Caches, t_now: float) -> SupervisorInputs:
        pl = self.cfg.platform
        inp = SupervisorInputs()

        imu_fresh = c.imu_rx is not None and (t_now - c.imu_rx) <= SENSOR_STALE_S
        inp.imu = IMUData(w=c.imu_w, valid=imu_fresh)

        v_enc = c.enc_speed if c.enc_speed is not None else ta.wheel_speed(
            c.w_rl, c.w_rr, pl.wheel_radius, pl.rear_left_wheel_sign, pl.rear_right_wheel_sign)
        enc_fresh = c.enc_rx is not None and (t_now - c.enc_rx) <= SENSOR_STALE_S
        inp.enc = EncoderData(v=v_enc, valid=enc_fresh and v_enc is not None)

        inp.cam_cones   = ConeObs(range_array=c.cam_range if c.cam_range is not None else np.zeros(0))
        inp.lidar_cones = ConeObs(range_array=c.lid_range if c.lid_range is not None else np.zeros(0))
        inp.lidar_scan  = LidarScan(ranges=c.scan_ranges if c.scan_ranges is not None else np.zeros(0))
        inp.ekf = EKFSnapshot(x=c.ekf_x, P=c.ekf_P, last_nis=c.ekf_last_nis, updated=c.ekf_updated)

        delta = ta.steering_cmd(c.d_fl, c.d_fr)
        if delta is None and c.mpc_delta is not None:
            delta = c.mpc_delta
        inp.delta_cmd = pl.steer_sign * (delta or 0.0)
        inp.steering_meas = c.steer_meas

        if pl.v_ref_source == "mpc_stats" and c.mpc_v_ref is not None:
            inp.v_ref = c.mpc_v_ref
        elif pl.v_ref_source == "trajectory" and c.traj_v is not None:
            inp.v_ref = c.traj_v
        else:
            inp.v_ref = 0.0

        if self._fg_on:
            fg = self.cfg.filter_gap
            with self._lock:
                res = ta.filter_gap(self._hist_local, self._hist_ref, fg.window_s,
                                    fg.pos_thresh_m, fg.yaw_thresh_rad, t_now, fg.max_age_s)
            inp.filter_gap = None if res is None else res[0]

        inp.cte           = c.cte
        inp.mpc_time_ms   = c.mpc_time_ms
        inp.mpc_budget_ms = c.mpc_budget_ms
        inp.throttle_cmd  = c.throttle_cmd
        return inp

    # ------------------------------------------------------------------
    #  Publishing helpers
    # ------------------------------------------------------------------
    def _make_header(self, stamp: rospy.Time, frame_id: str = "supervisor") -> Header:
        h = Header()
        h.stamp    = stamp
        h.frame_id = frame_id
        return h

    def _publish_residuals(self, now, r: np.ndarray, r_valid: np.ndarray) -> None:
        msg = ResidualVector()
        msg.header  = self._make_header(now)
        msg.r       = [float(x) for x in r]
        msg.r_valid = [bool(x) for x in r_valid]
        # Send names once per process lifetime — saves bandwidth at 20 Hz.
        if not self._names_published:
            msg.names             = list(self.cfg.residual.names)
            self._names_published = True
        else:
            msg.names = []
        self.pub_res.publish(msg)

    def _publish_posterior(self, now, mu_fine: np.ndarray, mu_coarse: np.ndarray) -> None:
        msg = ModePosterior()
        msg.header    = self._make_header(now)
        msg.mu_fine   = list(mu_fine)
        msg.mu_coarse = list(mu_coarse)
        msg.loglik    = list(self.imm.loglik)
        msg.mu_bar    = list(self.imm.mu_bar)
        self.pub_post.publish(msg)

    def _publish_flags(self, now, flags) -> None:
        msg = FaultFlags()
        msg.header              = self._make_header(now)
        msg.camera_timeout      = flags.camera_timeout
        msg.lidar_timeout       = flags.lidar_timeout
        msg.cones_cam_timeout   = flags.cones_cam_timeout
        msg.cones_lidar_timeout = flags.cones_lidar_timeout
        msg.cones_both_timeout  = flags.cones_both_timeout
        msg.imu_timeout         = flags.imu_timeout
        msg.encoder_timeout     = flags.encoder_timeout
        msg.steering_fault      = flags.steering_fault
        msg.drive_fault         = flags.drive_fault
        msg.ekf_divergence      = flags.ekf_divergence
        msg.cte_exceeded        = flags.cte_exceeded
        msg.speed_deviation     = flags.speed_deviation
        msg.mpc_overrun         = flags.mpc_overrun
        msg.obstacle_detected   = flags.obstacle_detected
        msg.battery_low         = flags.battery_low
        msg.overcurrent         = flags.overcurrent
        self.pub_flags.publish(msg)

    def _publish_action(self, now, act, mu_fine: np.ndarray, mu_coarse: np.ndarray) -> None:
        msg = SupervisorActionMsg()
        msg.header           = self._make_header(now)
        msg.mode_coarse      = int(act.mode_coarse)
        msg.mode_fine        = int(act.mode_fine)
        msg.mode_name        = act.mode_name
        msg.mode_name_fine   = act.mode_name_fine
        msg.speed_scale      = float(act.speed_scale)
        msg.estop            = bool(act.estop)
        msg.adjust_slam      = bool(act.adjust_slam)
        msg.fault_isolation  = act.fault_isolation
        msg.override_cause   = act.override_cause
        msg.severity         = float(act.severity)
        msg.mu_fine          = list(mu_fine)
        msg.mu_coarse        = list(mu_coarse)
        self.pub_act.publish(msg)


# =====================================================================
def main() -> None:
    node = SupervisorNode()
    rospy.loginfo("supervisor_node: ready.")
    rospy.spin()


if __name__ == "__main__":
    main()
