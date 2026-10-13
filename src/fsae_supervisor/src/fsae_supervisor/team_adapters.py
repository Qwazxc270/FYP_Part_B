"""Adapters from the team's (FYP_Part_B) ROS interfaces to the
supervisor's internal inputs.

ROS-agnostic on purpose: every function takes plain Python / numpy
values so it can be unit-tested without a ROS install. The node
(scripts/supervisor_node.py) pulls fields out of the ROS messages and
calls these.

Team interfaces consumed (root src/ of FYP_Part_B, guidance_gazebo.launch):

  /imu                                sensor_msgs/Imu            (Gazebo IMU plugin)
  /scan                               sensor_msgs/LaserScan      (Gazebo LiDAR plugin)
  /wheelrl_motor/velocity             std_msgs/Float32 [rad/s]   (gazebo_ros_motors)
  /wheelrr_motor/velocity             std_msgs/Float32 [rad/s]
  /qcar/base_fl_controller/command    std_msgs/Float64 [rad]     (MPC steering cmd)
  /qcar/base_fr_controller/command    std_msgs/Float64 [rad]
  /odometry/filtered                  nav_msgs/Odometry          (ekf_slam_node, map/odom frame)
  /cone_detections_camera             qcar_navigation/ConeDetectionArray
  /cone_detections_lidar              geometry_msgs/PoseArray    (lidar_cone_detector_node)
  /qcar/trajectory_topic              qcar_guidance/TrajectoryMessage (online_guidance_node)
"""
from __future__ import annotations

import math
from typing import Optional, Sequence, Tuple

import numpy as np


def wheel_speed(w_rl: Optional[float], w_rr: Optional[float],
                radius: float, sign_rl: float = -1.0, sign_rr: float = 1.0
                ) -> Optional[float]:
    """Longitudinal speed [m/s] from the rear wheel angular velocities.

    Mirrors nav_filter_node.py:  v = r * 0.5 * (sign_rl*w_rl + sign_rr*w_rr)
    (team default signs: rl = -1, rr = +1). If only one wheel has reported,
    that wheel alone is used.
    """
    vals = []
    if w_rl is not None and math.isfinite(w_rl):
        vals.append(sign_rl * float(w_rl))
    if w_rr is not None and math.isfinite(w_rr):
        vals.append(sign_rr * float(w_rr))
    if not vals:
        return None
    return float(radius * sum(vals) / len(vals))


def steering_cmd(d_fl: Optional[float], d_fr: Optional[float]) -> Optional[float]:
    """Equivalent bicycle steering command [rad] = mean of the two front
    steering commands that have been received."""
    vals = [float(d) for d in (d_fl, d_fr) if d is not None and math.isfinite(d)]
    if not vals:
        return None
    return float(sum(vals) / len(vals))


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return float(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def odometry_to_ekf(px: float, py: float, yaw: float, v: float,
                    pose_cov_6x6: Sequence[float]
                    ) -> Tuple[np.ndarray, Optional[np.ndarray]]:
    """nav_msgs/Odometry -> (x=[X, Y, yaw, v], P 4x4).

    P is built from the ROS 6x6 pose covariance (row-major, order
    x, y, z, roll, pitch, yaw). Only the x/y/yaw block is meaningful;
    the velocity variance is unknown and set to 0. sigma_pos (r1) only
    uses P[:2,:2]. Returns P=None if the covariance is missing/all-zero
    (r1 is then marked invalid).
    """
    x = np.array([px, py, yaw, v], dtype=float)
    c = np.asarray(pose_cov_6x6, dtype=float).reshape(-1)
    if c.size != 36 or not np.all(np.isfinite(c)) or not np.any(c):
        return x, None
    C = c.reshape(6, 6)
    idx = [0, 1, 5]
    P = np.zeros((4, 4))
    P[np.ix_([0, 1, 2], [0, 1, 2])] = C[np.ix_(idx, idx)]
    return x, P


def cone_ranges_from_points(xs: Sequence[float], ys: Sequence[float]) -> np.ndarray:
    """Body-frame cone positions -> ranges [m]. An empty array means
    "no cones this message" and lets the freshness timer grow."""
    xs = np.asarray(xs, dtype=float)
    ys = np.asarray(ys, dtype=float)
    if xs.size == 0:
        return np.zeros(0)
    r = np.hypot(xs, ys)
    return r[np.isfinite(r)]


def cone_ranges_from_detections(ranges: Sequence[float],
                                xs: Sequence[float], ys: Sequence[float]) -> np.ndarray:
    """qcar_navigation/ConeDetection[] -> ranges. Uses the detection's own
    ``range`` field when positive, else hypot(position.x, position.y)."""
    out = []
    for rg, x, y in zip(ranges, xs, ys):
        rg = float(rg)
        if not (math.isfinite(rg) and rg > 0):
            rg = math.hypot(float(x), float(y))
        if math.isfinite(rg):
            out.append(rg)
    return np.asarray(out, dtype=float)


def finite_scan_ranges(ranges: Sequence[float], range_min: float = 0.0) -> np.ndarray:
    """LaserScan.ranges -> finite returns at/above range_min (drops inf/nan
    and sub-min-range self-hits)."""
    a = np.asarray(ranges, dtype=float)
    a = a[np.isfinite(a)]
    if range_min > 0:
        a = a[a >= range_min]
    return a


# ---------------------------------------------------------------------
#  filter_gap: consistency between two pose estimators
# ---------------------------------------------------------------------
def _wrap(a: float) -> float:
    return float((a + math.pi) % (2.0 * math.pi) - math.pi)


class PoseHistory:
    """Time-stamped (t, x, y, yaw) ring with linear interpolation
    (yaw interpolated on the unwrapped difference)."""

    def __init__(self, horizon_s: float = 3.0) -> None:
        self.horizon = float(horizon_s)
        self.buf = []          # list of (t, x, y, yaw), ascending t

    def push(self, t: float, x: float, y: float, yaw: float) -> None:
        if self.buf and t <= self.buf[-1][0]:
            if t < self.buf[-1][0] - 1.0:      # clock jumped back (sim reset)
                self.buf.clear()
            else:
                return                          # duplicate / out-of-order
        self.buf.append((float(t), float(x), float(y), float(yaw)))
        t_min = t - self.horizon
        while len(self.buf) > 2 and self.buf[1][0] < t_min:
            self.buf.pop(0)

    @property
    def t_first(self) -> Optional[float]:
        return self.buf[0][0] if self.buf else None

    @property
    def t_last(self) -> Optional[float]:
        return self.buf[-1][0] if self.buf else None

    def at(self, t: float) -> Optional[Tuple[float, float, float]]:
        b = self.buf
        if not b or t < b[0][0] or t > b[-1][0]:
            return None
        for i in range(1, len(b)):
            if b[i][0] >= t:
                t0, x0, y0, h0 = b[i - 1]
                t1, x1, y1, h1 = b[i]
                a = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
                return (x0 + a * (x1 - x0), y0 + a * (y1 - y0),
                        _wrap(h0 + a * _wrap(h1 - h0)))
        return b[-1][1:]

    def clear(self) -> None:
        self.buf.clear()


def body_increment(p1, p2) -> Tuple[float, float, float]:
    """Pose increment p1 -> p2 expressed in p1's body frame:
    (dx_body, dy_body, dyaw). Invariant to a constant SE(2) offset
    between the two estimators' world frames."""
    dx, dy = p2[0] - p1[0], p2[1] - p1[1]
    c, s = math.cos(p1[2]), math.sin(p1[2])
    return (c * dx + s * dy, -s * dx + c * dy, _wrap(p2[2] - p1[2]))


def filter_gap(local: PoseHistory, ref: PoseHistory, window_s: float,
               pos_thresh_m: float, yaw_thresh_rad: float,
               t_now: Optional[float] = None, max_age_s: float = 0.5
               ) -> Optional[Tuple[float, float, float]]:
    """Relative-motion discrepancy between two estimators.

    Time alignment: both histories are interpolated at the SAME two
    instants t_end = min(last stamp of each) and t_start = t_end - window.
    Comparing body-frame increments over the window (not absolute
    poses) removes the slowly-varying offset between the drifting local
    frame and the SLAM frame, which is expected and not a fault.

    Returns (r, pos_err_m, yaw_err_rad) with
        r = max(pos_err / pos_thresh, |yaw_err| / yaw_thresh)
    or None if not computable (insufficient overlap or stale data).
    """
    if local.t_last is None or ref.t_last is None:
        return None
    t_end = min(local.t_last, ref.t_last)
    if t_now is not None and (t_now - t_end) > max_age_s:
        return None
    t_start = t_end - window_s
    pa1, pa2 = local.at(t_start), local.at(t_end)
    pb1, pb2 = ref.at(t_start), ref.at(t_end)
    if None in (pa1, pa2, pb1, pb2):
        return None
    ia, ib = body_increment(pa1, pa2), body_increment(pb1, pb2)
    pos_err = math.hypot(ia[0] - ib[0], ia[1] - ib[1])
    yaw_err = _wrap(ia[2] - ib[2])
    r = max(pos_err / max(pos_thresh_m, 1e-6), abs(yaw_err) / max(yaw_thresh_rad, 1e-6))
    return float(r), float(pos_err), float(yaw_err)
