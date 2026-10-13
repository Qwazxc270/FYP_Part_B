#!/usr/bin/env python3
"""Fault injector node.

Port of ``faultInjection.m`` and ``initFaultConfig.m``. Sits between
the team's topics and the SUPERVISOR ONLY (observation-side injection;
the team's EKF / guidance / MPC keep reading the real topics, so no
teammate node is modified or remapped). Topic names come from the
platform profile (config/platforms/gazebo.yaml):

    team topic                      fault injector     supervisor reads
    /imu                     --->   this node   --->   /supervisor_fi/imu
    /wheelr{l,r}_motor/velocity ->      "              /supervisor_fi/wheelr{l,r}_motor/velocity
    /cone_detections_camera  --->       "              /supervisor_fi/cone_detections_camera
    /cone_detections_lidar   --->       "              /supervisor_fi/cone_detections_lidar
    /scan                    --->       "              /supervisor_fi/scan

Limitation: because the team stack is not fed the faulty data, the
closed-loop *consequences* of a fault (EKF drift, mistracking) are not
reproduced — only the supervisor's detection path is exercised.

Activation modes:
  (1) Scripted — the preset in config/fault_injector.yaml activates
      faults at absolute ROS times measured from an arm event.
      Call the ``~arm`` service (or the ``~set_fault`` service with
      any subsystem) to start the scripted clock.
  (2) Runtime — the ``~set_fault`` service can add or cancel any
      subsystem fault at any time.

The two mechanisms coexist: runtime faults OR scripted faults can
assert a subsystem as faulty. For IMU/encoder with subtypes, the
runtime command wins.
"""

from __future__ import annotations

import copy
import threading
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np
import rospy

from sensor_msgs.msg import Imu, LaserScan
from std_srvs.srv    import Trigger, TriggerResponse

from std_msgs.msg      import Float32
from geometry_msgs.msg import PoseArray
from qcar_navigation.msg import ConeDetectionArray
from fsae_supervisor.srv import SetFault, SetFaultResponse


# =====================================================================
@dataclass
class SubsystemFault:
    """Runtime state for a single subsystem."""
    active:       bool  = False
    start_time:   float = float("inf")        # seconds since arm
    end_time:     float = float("inf")
    fault_type:   str   = ""
    value:        float = 0.0

    def is_live(self, t_rel: float) -> bool:
        return self.active and (self.start_time <= t_rel <= self.end_time)


@dataclass
class InjectorState:
    camera:   SubsystemFault = field(default_factory=SubsystemFault)
    lidar:    SubsystemFault = field(default_factory=SubsystemFault)
    imu:      SubsystemFault = field(default_factory=SubsystemFault)
    encoder:  SubsystemFault = field(default_factory=SubsystemFault)
    steering: SubsystemFault = field(default_factory=SubsystemFault)
    drive:    SubsystemFault = field(default_factory=SubsystemFault)
    obstacle: SubsystemFault = field(default_factory=SubsystemFault)

    # Persisted runtime data (frozen values for freeze-type faults)
    frozen_imu_w: Optional[float] = None
    frozen_w_rl: Optional[float] = None
    frozen_w_rr: Optional[float] = None


# =====================================================================
class FaultInjectorNode:
    _SUBSYSTEMS = ("camera", "lidar", "imu", "encoder", "steering", "drive", "obstacle")

    def __init__(self) -> None:
        rospy.init_node("fault_injector_node")

        ns = rospy.get_param("~config_ns", "fault_injector")
        p  = rospy.get_param(ns)

        # Merge the default block with the chosen preset
        preset_name = str(p.get("preset", "none"))
        state = self._build_state(p, preset_name)
        self._state: InjectorState = state
        self._lock  = threading.Lock()

        # Arm time: set when the first scripted fault starts ticking.
        # (Set at ~arm service call, or implicitly on node start.)
        self._armed_at: Optional[rospy.Time] = None
        auto_arm = bool(rospy.get_param("~auto_arm", True))
        if auto_arm:
            self._armed_at = rospy.Time.now()

        # NOTE: steering / drive faults are tracked in state but have no
        # effect on the gazebo profile: there is no steering feedback or
        # MpcStats to corrupt, and steer_parity is disabled. Camera, LiDAR,
        # IMU, encoder and obstacle faults are fully supported.
        t = p["topics"]
        # ---- Publishers first (callbacks use them) ----
        self.pub_imu       = rospy.Publisher(t["imu_out"],         Imu,                queue_size=50)
        self.pub_w_rl      = rospy.Publisher(t["wheel_rl_out"],    Float32,            queue_size=50)
        self.pub_w_rr      = rospy.Publisher(t["wheel_rr_out"],    Float32,            queue_size=50)
        self.pub_cam_cones = rospy.Publisher(t["cones_cam_out"],   ConeDetectionArray, queue_size=20)
        self.pub_lid_cones = rospy.Publisher(t["cones_lidar_out"], PoseArray,          queue_size=20)
        self.pub_scan      = rospy.Publisher(t["lidar_scan_out"],  LaserScan,          queue_size=5)

        # ---- Subscribers (team topics) ----
        rospy.Subscriber(t["imu_in"],         Imu,                self._cb_imu,        queue_size=50)
        rospy.Subscriber(t["wheel_rl_in"],    Float32,            self._cb_w_rl,       queue_size=50)
        rospy.Subscriber(t["wheel_rr_in"],    Float32,            self._cb_w_rr,       queue_size=50)
        rospy.Subscriber(t["cones_cam_in"],   ConeDetectionArray, self._cb_cam_cones,  queue_size=20)
        rospy.Subscriber(t["cones_lidar_in"], PoseArray,          self._cb_lid_cones,  queue_size=20)
        rospy.Subscriber(t["lidar_scan_in"],  LaserScan,          self._cb_lidar_scan, queue_size=5)

        # ---- Services ----
        rospy.Service("~set_fault", SetFault, self._on_set_fault)
        rospy.Service("~arm",       Trigger,  self._on_arm)

        rospy.loginfo("fault_injector: preset='%s' (armed=%s, auto_arm=%s)",
                      preset_name, self._armed_at is not None, auto_arm)

    # ------------------------------------------------------------------
    def _t_rel(self) -> float:
        if self._armed_at is None:
            return float("-inf")
        return (rospy.Time.now() - self._armed_at).to_sec()

    def _live(self, subsystem: str) -> SubsystemFault:
        """Return the live fault for a subsystem (or an inactive default)."""
        f = getattr(self._state, subsystem)
        return f if f.is_live(self._t_rel()) else SubsystemFault()

    # ------------------------------------------------------------------
    #  Callbacks — apply injection, then republish
    # ------------------------------------------------------------------
    def _cb_imu(self, msg: Imu) -> None:
        with self._lock:
            f = self._live("imu")
            if f.active:
                if f.fault_type == "freeze":
                    if self._state.frozen_imu_w is None:
                        self._state.frozen_imu_w = float(msg.angular_velocity.z)
                    msg.angular_velocity.z = self._state.frozen_imu_w
                elif f.fault_type == "bias":
                    msg.angular_velocity.z = float(msg.angular_velocity.z) + f.value
                elif f.fault_type == "noise":
                    msg.angular_velocity.z = float(msg.angular_velocity.z) + f.value * float(np.random.randn())
            else:
                self._state.frozen_imu_w = None
        self.pub_imu.publish(msg)

    def _enc_fault(self, w: float, attr: str) -> Optional[float]:
        """Apply the encoder fault to one wheel. Returns the value to
        publish, or None to drop the message (encoder 'zero'/dropout is
        modelled as publishing 0.0 so the freeze detector sees it)."""
        f = self._live("encoder")
        if not f.active:
            setattr(self._state, attr, None)
            return w
        if f.fault_type == "freeze":
            if getattr(self._state, attr) is None:
                setattr(self._state, attr, w)
            return getattr(self._state, attr)
        if f.fault_type == "zero":
            return 0.0
        if f.fault_type == "scale":
            return w * (f.value if f.value != 0 else 1.0)
        return w

    def _cb_w_rl(self, msg: Float32) -> None:
        with self._lock:
            msg.data = self._enc_fault(float(msg.data), "frozen_w_rl")
        self.pub_w_rl.publish(msg)

    def _cb_w_rr(self, msg: Float32) -> None:
        with self._lock:
            msg.data = self._enc_fault(float(msg.data), "frozen_w_rr")
        self.pub_w_rr.publish(msg)

    def _cb_cam_cones(self, msg: ConeDetectionArray) -> None:
        with self._lock:
            if self._live("camera").active:
                msg.detections = []
        self.pub_cam_cones.publish(msg)

    def _cb_lid_cones(self, msg: PoseArray) -> None:
        with self._lock:
            if self._live("lidar").active:
                msg.poses = []
        self.pub_lid_cones.publish(msg)

    def _cb_lidar_scan(self, msg: LaserScan) -> None:
        """LiDAR dropout suppresses the whole scan; obstacle injection
        overwrites a sector to simulate a close obstacle."""
        with self._lock:
            if self._live("lidar").active:
                # Dropout: publish an empty-range scan
                msg.ranges = []
            elif self._live("obstacle").active:
                # Inject the closest obstacle at the nominal min distance
                # so the supervisor's hard-override fires. More faithful
                # geometry (car pose, obstacle world XY → scan bearing)
                # belongs in a Gazebo plugin; this shim does the minimum
                # needed to exercise the OBS-01 test path.
                arr = list(msg.ranges)
                if arr:
                    # 10 cm — under obstacle_min_dist = 0.15 m
                    mid = len(arr) // 2
                    arr[mid] = 0.10
                    msg.ranges = arr
        self.pub_scan.publish(msg)

    # ------------------------------------------------------------------
    #  Services
    # ------------------------------------------------------------------
    def _on_set_fault(self, req: SetFault) -> SetFaultResponse:
        r = SetFaultResponse()
        sub = (req.subsystem or "").strip().lower()
        if sub not in self._SUBSYSTEMS:
            r.success = False
            r.message = f"unknown subsystem '{sub}'  (valid: {', '.join(self._SUBSYSTEMS)})"
            return r
        with self._lock:
            if self._armed_at is None:
                self._armed_at = rospy.Time.now()
            f: SubsystemFault = getattr(self._state, sub)
            if req.active:
                t_rel = self._t_rel()
                f.active     = True
                f.start_time = t_rel
                f.end_time   = t_rel + req.duration_s if req.duration_s > 0 else float("inf")
                f.fault_type = req.type or f.fault_type
                f.value      = float(req.value)
                r.message    = f"{sub} fault ON (type={f.fault_type}, value={f.value}, duration={req.duration_s}s)"
            else:
                f.active   = False
                r.message  = f"{sub} fault OFF"
        rospy.loginfo("fault_injector: %s", r.message)
        r.success = True
        return r

    def _on_arm(self, _req) -> TriggerResponse:
        self._armed_at = rospy.Time.now()
        rospy.loginfo("fault_injector: armed at %s", self._armed_at.to_sec())
        return TriggerResponse(success=True, message="armed")

    # ------------------------------------------------------------------
    #  Config parsing
    # ------------------------------------------------------------------
    @classmethod
    def _build_state(cls, p: dict, preset_name: str) -> InjectorState:
        """Merge default per-subsystem block with preset."""
        merged: Dict[str, dict] = {sub: copy.deepcopy(p.get(sub, {})) for sub in cls._SUBSYSTEMS}

        presets = p.get("presets", {}) or {}
        if preset_name and preset_name != "none":
            if preset_name not in presets:
                rospy.logwarn("fault_injector: preset '%s' not in config — using none", preset_name)
            else:
                for sub, sub_cfg in presets[preset_name].items():
                    if sub in merged:
                        merged[sub].update(sub_cfg)

        state = InjectorState()
        for sub in cls._SUBSYSTEMS:
            f = getattr(state, sub)
            cfg = merged[sub]
            f.active      = bool(cfg.get("active", False))
            f.start_time  = float(cfg.get("start_time", float("inf")))
            f.end_time    = float(cfg.get("end_time",   float("inf")))
            f.fault_type  = str(cfg.get("type", ""))
            # subtype-specific default values
            if sub == "imu":
                if f.fault_type == "bias":
                    f.value = float(cfg.get("bias_value", 0.0))
                elif f.fault_type == "noise":
                    f.value = float(cfg.get("noise_sigma", 0.0))
            elif sub == "encoder" and f.fault_type == "scale":
                f.value = float(cfg.get("scale_factor", 1.0))
        return state


def main() -> None:
    FaultInjectorNode()
    rospy.loginfo("fault_injector_node: ready.")
    rospy.spin()


if __name__ == "__main__":
    main()
