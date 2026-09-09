#! /usr/bin/env python3
"""
Online guidance node -- FYP Part B (Thiha "Thomas" Thet Zaw)
=============================================================

Plans a local path ONLINE from live fused cone detections, instead of
loading a pre-known track map (the Part A limitation this fixes).

Pipeline position:
    qcar_navigation (camera+lidar fusion)  -->  THIS NODE  -->  qcar_control

Subscribes:
    /cone_detections_fused_coloured  (qcar_navigation/ConeDetectionArray)
    /odometry/filtered               (nav_msgs/Odometry)
Publishes:
    /qcar/trajectory_topic           (qcar_guidance/TrajectoryMessage)

Features:
  * Cone memory      -- cones persist briefly after last seen and merge
                        across frames, preventing path jitter on dropouts.
  * Frame handling   -- detections arriving in the car frame are transformed
                        into the fixed odom frame before being remembered.
  * Speed profiling  -- per-waypoint curvature slows the car into corners.
  * Mode machine     -- CREEP (bootstrap, track not fully visible yet)
                        <-> TRACK (full midpoint planning).

All tunables load from config/guidance_params.yaml via private rosparams.
"""

import rospy
import math
import numpy as np
from scipy.interpolate import interp1d
from qcar_guidance.msg import TrajectoryMessage
from qcar_navigation.msg import ConeDetectionArray, ConeDetection
from nav_msgs.msg import Odometry

latest_cones = None
current_x = None
current_y = None
current_yaw = None


class Params:
    """Loads all tunables from the parameter server (guidance_params.yaml)."""

    def __init__(self):
        gp = rospy.get_param
        self.max_velocity = gp("~max_velocity", 0.5)
        self.min_velocity = gp("~min_velocity", 0.2)
        self.creep_velocity = gp("~creep_velocity", 0.15)
        self.min_cones_per_side = gp("~min_cones_per_side", 2)
        self.track_half_width = gp("~track_half_width", 1.5)
        self.creep_distance = gp("~creep_distance", 1.5)
        self.cone_memory_seconds = gp("~cone_memory_seconds", 3.0)
        self.cone_merge_dist = gp("~cone_merge_dist", 0.5)
        self.max_planning_range = gp("~max_planning_range", 6.0)
        self.curvature_slowdown = gp("~curvature_slowdown", 2.0)
        self.plan_rate_hz = gp("~plan_rate_hz", 2)
        self.detections_in_car_frame = gp("~detections_in_car_frame", True)


class ConeMemory:
    """Short-term memory of cones in the FIXED (odom) frame.

    Merges repeated sightings of the same cone and forgets cones not seen
    for cone_memory_seconds, so momentary detection dropouts don't cause
    the planned path to jitter frame-to-frame.
    """

    def __init__(self, p):
        self.p = p
        self.cones = []  # dicts: {x, y, colour, last_seen}

    def update(self, detections, now, car_x, car_y, car_yaw):
        cy, sy = math.cos(car_yaw), math.sin(car_yaw)
        for det in detections:
            if det.colour not in (ConeDetection.BLUE, ConeDetection.YELLOW):
                continue
            if self.p.detections_in_car_frame:
                # rotate+translate car-frame detection into the odom frame
                x = car_x + cy * det.position.x - sy * det.position.y
                y = car_y + sy * det.position.x + cy * det.position.y
            else:
                x, y = det.position.x, det.position.y

            merged = False
            for c in self.cones:
                if (c['colour'] == det.colour and
                        math.dist((x, y), (c['x'], c['y'])) < self.p.cone_merge_dist):
                    c['x'], c['y'] = x, y
                    c['last_seen'] = now
                    merged = True
                    break
            if not merged:
                self.cones.append({'x': x, 'y': y,
                                   'colour': det.colour, 'last_seen': now})

        self.cones = [c for c in self.cones
                      if (now - c['last_seen']) < self.p.cone_memory_seconds]

    def get_sides(self, car_x, car_y):
        blue, yellow = [], []
        for c in self.cones:
            if math.dist((car_x, car_y), (c['x'], c['y'])) > self.p.max_planning_range:
                continue
            pt = (c['x'], c['y'])
            (blue if c['colour'] == ConeDetection.BLUE else yellow).append(pt)
        blue.sort(key=lambda q: math.dist((car_x, car_y), q))
        yellow.sort(key=lambda q: math.dist((car_x, car_y), q))
        return blue, yellow


def cones_callback(msg):
    global latest_cones
    latest_cones = msg


def odom_callback(msg):
    global current_x, current_y, current_yaw
    current_x = msg.pose.pose.position.x
    current_y = msg.pose.pose.position.y
    q = msg.pose.pose.orientation
    current_yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                             1 - 2 * (q.y * q.y + q.z * q.z))


def _dedupe(path):
    """Drop duplicate consecutive points -- zero-length segments break interp1d."""
    keep = [0]
    for i in range(1, path.shape[1]):
        if math.dist(path[:, i], path[:, keep[-1]]) > 1e-6:
            keep.append(i)
    return path[:, keep]


def _resample(path, points_per_metre=8, min_points=3):
    """Fit a quadratic spline through the path and resample it evenly."""
    path = _dedupe(path)
    if path.shape[1] < 3:
        return path
    d = np.cumsum(np.sqrt(np.sum(np.diff(path, axis=1) ** 2, axis=0)))
    d = np.insert(d, 0, 0)
    if d[-1] == 0:
        return path
    n = max(int(round(points_per_metre * d[-1])), min_points)
    alpha = np.linspace(0, 1, n)
    return interp1d(d / d[-1], path, kind='quadratic', axis=1)(alpha)


def build_midpoint_path(blue, yellow, car_x, car_y):
    """TRACK mode: pair nearest blue/yellow cones into midpoints, spline through.

    Nearest-index pairing is a deliberate simple heuristic; Delaunay
    triangulation would be the more robust upgrade (noted as future work).
    """
    n_pairs = min(len(blue), len(yellow))
    pts = [[car_x], [car_y]]
    for i in range(n_pairs):
        pts[0].append((blue[i][0] + yellow[i][0]) / 2)
        pts[1].append((blue[i][1] + yellow[i][1]) / 2)
    return _resample(np.array(pts))


def build_creep_path(blue, yellow, car_x, car_y, car_yaw, p):
    """CREEP mode: not enough of the track visible yet -- nudge forward.

    Aims toward the assumed track centre offset from whichever cone line IS
    visible (blue = left boundary, yellow = right boundary, FSAE convention),
    or straight ahead if nothing usable is visible.

    NOTE: the path is linearly interpolated to 6 waypoints (not just
    start+target). Downstream controllers commonly fit splines through
    received waypoints, and spline fits need >= 3-4 points -- publishing a
    2-point path can crash them.
    """
    fwd = (math.cos(car_yaw), math.sin(car_yaw))
    right = (math.sin(car_yaw), -math.cos(car_yaw))
    left = (-right[0], -right[1])

    if blue and not yellow:
        rx, ry = blue[0]
        tx = rx + right[0] * p.track_half_width
        ty = ry + right[1] * p.track_half_width
    elif yellow and not blue:
        rx, ry = yellow[0]
        tx = rx + left[0] * p.track_half_width
        ty = ry + left[1] * p.track_half_width
    else:
        tx = car_x + fwd[0] * p.creep_distance
        ty = car_y + fwd[1] * p.creep_distance

    n = 6
    xs = np.linspace(car_x, tx, n)
    ys = np.linspace(car_y, ty, n)
    return _dedupe(np.array([xs, ys]))


def path_curvatures(path):
    """Approximate curvature at each waypoint: heading change / segment length."""
    n = path.shape[1]
    curv = np.zeros(n)
    for i in range(1, n - 1):
        v1 = path[:, i] - path[:, i - 1]
        v2 = path[:, i + 1] - path[:, i]
        a1 = math.atan2(v1[1], v1[0])
        a2 = math.atan2(v2[1], v2[0])
        dang = abs(math.atan2(math.sin(a2 - a1), math.cos(a2 - a1)))
        curv[i] = dang / (np.linalg.norm(v1) + 1e-6)
    if n > 2:
        curv[0], curv[-1] = curv[1], curv[-2]
    return curv


def build_trajectory_message(path, base_velocity, speeds=None):
    """Encode a (possibly speed-profiled) path as a TrajectoryMessage.

    TrajectoryMessage carries one scalar velocity, so the varying speed
    profile is encoded through waypoint_times: slower segments get longer
    time allocations, which a time-tracking controller naturally follows.
    """
    msg = TrajectoryMessage()
    n = path.shape[1]
    if speeds is None:
        speeds = np.full(n, base_velocity)
    times = [0.0]
    for i in range(1, n):
        d = math.dist([path[0, i - 1], path[1, i - 1]],
                      [path[0, i], path[1, i]])
        v = max((speeds[i - 1] + speeds[i]) / 2.0, 0.05)
        times.append(times[-1] + d / v)
    msg.waypoint_times = times
    msg.waypoint_x = path[0]
    msg.waypoint_y = path[1]
    msg.velocity = float(np.mean(speeds))
    return msg


def main():
    global latest_cones
    rospy.init_node('online_guidance_node')
    p = Params()
    memory = ConeMemory(p)
    mode = "CREEP"

    rospy.Subscriber('/cone_detections_fused_coloured',
                     ConeDetectionArray, cones_callback)
    rospy.Subscriber('/odometry/filtered', Odometry, odom_callback)
    traj_pub = rospy.Publisher('/qcar/trajectory_topic',
                               TrajectoryMessage, queue_size=1)

    rospy.loginfo("online_guidance_node started (car_frame_detections=%s)",
                  p.detections_in_car_frame)

    rate = rospy.Rate(p.plan_rate_hz)
    while not rospy.is_shutdown():
        if latest_cones is None or current_x is None:
            rospy.loginfo_throttle(
                2, "[%s] Waiting for cone detections and odometry...", mode)
            rate.sleep()
            continue

        now = rospy.get_time()
        memory.update(latest_cones.detections, now,
                      current_x, current_y, current_yaw)
        blue, yellow = memory.get_sides(current_x, current_y)

        if (len(blue) >= p.min_cones_per_side and
                len(yellow) >= p.min_cones_per_side):
            if mode != "TRACK":
                rospy.loginfo("Mode change: %s -> TRACK", mode)
                mode = "TRACK"
            path = build_midpoint_path(blue, yellow, current_x, current_y)
            curv = path_curvatures(path)
            speeds = np.clip(
                p.max_velocity / (1.0 + p.curvature_slowdown * curv),
                p.min_velocity, p.max_velocity)
            traj_pub.publish(build_trajectory_message(path, p.max_velocity, speeds))
            rospy.loginfo(
                "[TRACK] %d waypoints | mem: blue=%d yellow=%d | v: %.2f-%.2f m/s",
                path.shape[1], len(blue), len(yellow),
                float(np.min(speeds)), float(np.max(speeds)))
        else:
            if mode != "CREEP":
                rospy.loginfo("Mode change: %s -> CREEP", mode)
                mode = "CREEP"
            path = build_creep_path(blue, yellow,
                                    current_x, current_y, current_yaw, p)
            traj_pub.publish(build_trajectory_message(path, p.creep_velocity))
            rospy.loginfo_throttle(
                2, "[CREEP] nudging forward | mem: blue=%d yellow=%d",
                len(blue), len(yellow))
        rate.sleep()


if __name__ == '__main__':
    main()
