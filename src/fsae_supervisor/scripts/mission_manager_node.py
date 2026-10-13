#!/usr/bin/env python3
"""Mission manager node.

Ports the FSM in ``initMission.m`` / ``updateMission.m`` /
``getMissionTargetSpeed.m`` to ROS. Replaces the MATLAB keyboard
callback (SPACE = go, ESC = estop) with a ``SetMission`` service.

States (match MissionStatus.msg):
    1 STATE_WAITING    — waiting for /mission/set_mission go
    2 STATE_RUNNING    — actively running the lap or timed mode
    3 STATE_FINISHED   — lap target hit or time limit reached
    4 STATE_ESTOPPED   — latching; supervisor Critical, manual estop, or reset

Outputs:
    /mission/status         (fsae_supervisor/MissionStatus)   @ rate_hz
    /mission/target_speed   (std_msgs/Float32)                pre-supervisor scaling
"""

from __future__ import annotations

import threading
from typing import Optional

import rospy

import json

from std_msgs.msg import UInt32, Float32, Header, String

from fsae_supervisor.msg import MissionStatus, SupervisorAction as SupervisorActionMsg
from fsae_supervisor.srv import SetMission, SetMissionResponse


# Mission FSM states (keep in sync with MissionStatus.msg constants).
STATE_WAITING  = 1
STATE_RUNNING  = 2
STATE_FINISHED = 3
STATE_ESTOPPED = 4
_STATE_NAMES = {1: "WaitingForGo", 2: "Running", 3: "Finished", 4: "EStopped"}


class MissionManager:
    def __init__(self) -> None:
        rospy.init_node("mission_manager_node")

        # Bug fix: the port read rospy.get_param("~") first, which (with
        # full_stack.launch's private rosparam load) returned a dict
        # nested under 'mission_manager' and crashed on p["rate_hz"].
        # Read the global namespace named by ~config_ns instead.
        ns = rospy.get_param("~config_ns", "mission_manager")
        p = rospy.get_param(ns)
        self.rate_hz          = float(p["rate_hz"])
        self.mode             = str(p["mode"])
        self.lap_count_target = int(p.get("lap_count_target", 3))
        self.lap_target_speed = float(p.get("lap_target_speed", 0.7))
        self.time_limit_s     = float(p.get("time_limit_s", 60.0))
        self.time_target_speed = float(p.get("time_target_speed", 1.0))
        self.wrap_high_frac   = float(p.get("wrap_high_frac", 0.85))
        self.wrap_low_frac    = float(p.get("wrap_low_frac", 0.15))
        self.min_lap_time_s   = float(p.get("min_lap_time_s", 3.0))
        self.waypoints_per_lap = int(p.get("waypoints_per_lap_default", 0))

        topics = p["topics"]
        self._in_wp_idx   = topics.get("waypoint_index_in", "") or ""
        self._in_wp_per   = topics.get("waypoints_per_lap_in", "") or ""
        self._in_gstatus  = topics.get("guidance_status_in", "") or ""
        self.lap_source   = str(p.get("lap_source", "guidance_status"))
        self._guid_laps_done = None   # type: Optional[int]
        self._guid_laps_at_go = 0
        self._in_action   = topics["action_in"]
        self._out_status  = topics["status_out"]
        self._out_target  = topics["target_speed_out"]

        # -------- State machine --------
        self._lock = threading.Lock()
        self.state       = STATE_WAITING
        self.current_lap = 0
        self.start_time  = None              # type: Optional[rospy.Time]
        self.last_lap_time = None            # type: Optional[rospy.Time]
        self.prev_wp_idx = 0
        self.latest_wp_idx = 0
        self._estop_latched = False

        # -------- ROS I/O --------
        self.pub_status = rospy.Publisher(self._out_status, MissionStatus, queue_size=10, latch=True)
        self.pub_target = rospy.Publisher(self._out_target, Float32,       queue_size=10, latch=True)

        # Subscribers. (Bug fix: the port passed latch=True to a Subscriber,
        # which rospy rejects with TypeError. Latching is a publisher-side
        # property; a subscriber receives the last latched message on
        # connect automatically.)
        if self._in_wp_idx:
            rospy.Subscriber(self._in_wp_idx, UInt32, self._cb_wp,     queue_size=20)
        if self._in_wp_per:
            rospy.Subscriber(self._in_wp_per, UInt32, self._cb_wp_per, queue_size=5)
        if self._in_gstatus:
            rospy.Subscriber(self._in_gstatus, String, self._cb_gstatus, queue_size=5)
        if self.lap_source == "guidance_status" and not self._in_gstatus:
            rospy.logwarn("mission_manager: lap_source=guidance_status but no guidance_status_in topic; laps will not count")
        rospy.Subscriber(self._in_action, SupervisorActionMsg,   self._cb_action,  queue_size=5)

        rospy.Service("~set_mission", SetMission, self._on_set_mission)

        self._timer = rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self._on_tick)
        rospy.loginfo("mission_manager: mode=%s  rate=%.1fHz  target=%d laps / %.0fs",
                      self.mode, self.rate_hz, self.lap_count_target, self.time_limit_s)

    # ------------------------------------------------------------------
    def _target_speed_cmd(self) -> float:
        if self.mode == "lap":
            return self.lap_target_speed
        if self.mode == "time":
            return self.time_target_speed
        return 0.0

    # ------------------------------------------------------------------
    #  Callbacks
    # ------------------------------------------------------------------
    def _cb_wp(self, msg: UInt32) -> None:
        with self._lock:
            self.latest_wp_idx = int(msg.data)

    def _cb_wp_per(self, msg: UInt32) -> None:
        with self._lock:
            self.waypoints_per_lap = int(msg.data)

    def _cb_gstatus(self, msg: String) -> None:
        try:
            st = json.loads(msg.data)
            done = int(st.get("laps_done", st.get("lap", 1) - 1))
        except (ValueError, TypeError, AttributeError):
            return
        with self._lock:
            self._guid_laps_done = done

    def _cb_action(self, msg: SupervisorActionMsg) -> None:
        if msg.estop:
            with self._lock:
                if self.state != STATE_ESTOPPED:
                    rospy.logwarn("mission_manager: supervisor requested E-STOP — %s",
                                  msg.override_cause or msg.mode_name_fine)
                    self.state = STATE_ESTOPPED
                    self._estop_latched = True

    # ------------------------------------------------------------------
    #  Service: go / estop / reset
    # ------------------------------------------------------------------
    def _on_set_mission(self, req: SetMission) -> SetMissionResponse:
        cmd = (req.command or "").strip().lower()
        r   = SetMissionResponse()
        with self._lock:
            if cmd == "go":
                if self.state != STATE_WAITING:
                    r.success = False
                    r.message = f"cannot go from state {self.state}"
                else:
                    self.state = STATE_RUNNING
                    self.start_time    = rospy.Time.now()
                    self.last_lap_time = self.start_time
                    self.current_lap   = 0
                    self._guid_laps_at_go = self._guid_laps_done or 0
                    r.success = True
                    r.message = "mission: RUNNING"
                    rospy.loginfo("mission_manager: GO — starting mission")
            elif cmd == "estop":
                self.state = STATE_ESTOPPED
                self._estop_latched = True
                r.success = True
                r.message = "mission: ESTOPPED"
                rospy.logwarn("mission_manager: manual E-STOP")
            elif cmd == "reset":
                if self.state in (STATE_ESTOPPED, STATE_FINISHED):
                    self.state = STATE_WAITING
                    self.current_lap = 0
                    self.start_time = None
                    self.last_lap_time = None
                    self._estop_latched = False
                    r.success = True
                    r.message = "mission: WAITING" + self._reset_supervisor()
                    rospy.loginfo("mission_manager: RESET — waiting for go")
                else:
                    r.success = False
                    r.message = "reset only valid from Finished/EStopped"
            else:
                r.success = False
                r.message = f"unknown command '{cmd}'  (use: go | estop | reset)"
            r.new_state = self.state
        return r

    def _reset_supervisor(self) -> str:
        """Also clear the supervisor's latched Critical; otherwise its next
        estop=True message immediately re-latches the FSM."""
        from std_srvs.srv import Trigger
        srv = rospy.get_param("~supervisor_reset_srv", "/supervisor_node/reset")
        try:
            rospy.wait_for_service(srv, timeout=0.5)
            rospy.ServiceProxy(srv, Trigger)()
            return " (supervisor reset)"
        except Exception as e:      # noqa: BLE001
            rospy.logwarn("mission_manager: supervisor reset failed: %s", e)
            return " (WARNING: supervisor reset failed)"

    # ------------------------------------------------------------------
    #  Periodic update — implements updateMission.m
    # ------------------------------------------------------------------
    def _on_tick(self, _evt) -> None:
        now = rospy.Time.now()
        with self._lock:
            if self.state == STATE_RUNNING:
                # Lap counting, source 1: guidance node's own lap counter
                if self.lap_source == "guidance_status" and self._guid_laps_done is not None:
                    laps = max(0, self._guid_laps_done - self._guid_laps_at_go)
                    long_enough = self.last_lap_time is not None \
                                  and (now - self.last_lap_time).to_sec() >= self.min_lap_time_s
                    if laps > self.current_lap and long_enough:
                        self.current_lap   = laps
                        self.last_lap_time = now
                        rospy.loginfo("mission_manager: lap %d/%d",
                                      self.current_lap, self.lap_count_target)
                        if self.mode == "lap" and self.current_lap >= self.lap_count_target:
                            self.state = STATE_FINISHED

                # Lap counting, source 2: waypoint wrap (mirrors MATLAB exactly)
                if self.lap_source == "waypoint_wrap" and self.waypoints_per_lap > 0:
                    wrap_hi = int(round(self.wrap_high_frac * self.waypoints_per_lap))
                    wrap_lo = int(round(self.wrap_low_frac  * self.waypoints_per_lap))
                    lap_wrapped = (self.prev_wp_idx >= wrap_hi and self.latest_wp_idx <= wrap_lo)
                    long_enough = self.last_lap_time is not None \
                                  and (now - self.last_lap_time).to_sec() >= self.min_lap_time_s
                    if lap_wrapped and long_enough:
                        self.current_lap  += 1
                        self.last_lap_time = now
                        rospy.loginfo("mission_manager: lap %d/%d",
                                      self.current_lap, self.lap_count_target)
                        if self.mode == "lap" and self.current_lap >= self.lap_count_target:
                            self.state = STATE_FINISHED

                if self.mode == "time" and self.start_time is not None:
                    if (now - self.start_time).to_sec() >= self.time_limit_s:
                        self.state = STATE_FINISHED
                        rospy.loginfo("mission_manager: time limit reached")

                self.prev_wp_idx = self.latest_wp_idx

            # Snapshot for publishing
            state      = self.state
            current    = self.current_lap
            run_time   = (now - self.start_time).to_sec() if self.start_time else 0.0
            last_lap_t = (self.last_lap_time.to_sec() - self.start_time.to_sec()) \
                         if (self.last_lap_time and self.start_time) else 0.0

        # Pre-supervisor target speed — supervisor scales this downstream.
        target_speed = self._target_speed_cmd() if state == STATE_RUNNING else 0.0

        # ---- Publish status ----
        st = MissionStatus()
        st.header           = Header(stamp=now, frame_id="mission")
        st.state            = state
        st.state_name       = _STATE_NAMES.get(state, "?")
        st.mode             = self.mode
        st.current_lap      = current
        st.lap_count_target = self.lap_count_target if self.mode == "lap" else 0
        st.run_time_s       = run_time
        st.time_limit_s     = self.time_limit_s if self.mode == "time" else float("inf")
        st.target_speed_cmd = target_speed
        st.last_lap_time_s  = last_lap_t
        self.pub_status.publish(st)

        self.pub_target.publish(Float32(target_speed))


def main() -> None:
    MissionManager()
    rospy.loginfo("mission_manager_node: ready.")
    rospy.spin()


if __name__ == "__main__":
    main()
