#! /usr/bin/env python3
"""
online_guidance_node -- ROS wrapper for guidance_core (FYP Part B, Thomas).

All planning logic lives in guidance_core.py (no ROS), which is exactly the
code exercised by extras/offline_test.py. This node only:
  * converts ROS messages <-> plain Python data
  * compensates sensor latency: each detection message is placed using the
    car's LOCAL pose at the message's capture stamp, not the current pose
  * feeds SLAM pose pairs to the map->local alignment
  * publishes the trajectory, a status string, and a race-line marker

Subscribes (topic names are parameters)
  ~coloured_topic    /cone_detections_fused_coloured  ConeDetectionArray
  ~plain_topic       /cone_detections_fused           PoseArray
  ~local_odom_topic  /odometry/local                  Odometry (smooth, drifts)
  ~map_odom_topic    /odometry/filtered               Odometry (EKF-SLAM)
  ~map_markers_topic /cone_map_markers                MarkerArray (map + colour)
  ~map_topic         /cone_map                        PoseArray (map, no colour)
Publishes
  ~trajectory_topic  /qcar/trajectory_topic           TrajectoryMessage
  /guidance_status   std_msgs/String (JSON)
  /guidance_race_line visualization_msgs/Marker
"""

import json
import os
import sys
import threading
from collections import deque

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rospy                                                   # noqa: E402
from geometry_msgs.msg import PoseArray, Point                 # noqa: E402
from nav_msgs.msg import Odometry                              # noqa: E402
from std_msgs.msg import String                                # noqa: E402
from visualization_msgs.msg import Marker, MarkerArray         # noqa: E402
from qcar_guidance.msg import TrajectoryMessage                # noqa: E402
from qcar_navigation.msg import ConeDetectionArray             # noqa: E402

import guidance_core as gc                                     # noqa: E402


def yaw_of(q):
    import math
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def pose_of(msg):
    p = msg.pose.pose
    return (p.position.x, p.position.y, yaw_of(p.orientation))


def marker_colour(m):
    """EKF-SLAM encodes each landmark's colour vote as the marker RGB."""
    r, g, b = m.color.r, m.color.g, m.color.b
    if b > 0.7 and r < 0.3:
        return gc.BLUE
    if r > 0.8 and g > 0.7 and b < 0.3:
        return gc.YELLOW
    if r > 0.8 and 0.2 < g < 0.6 and b < 0.3:
        return gc.ORANGE
    return gc.UNKNOWN


class GuidanceNode:
    def __init__(self):
        params = {k: rospy.get_param('~' + k, v) for k, v in gc.DEFAULTS.items()}
        self.planner = gc.GuidancePlanner(params, self._log)
        self.p = self.planner.p
        self.lock = threading.Lock()

        gp = rospy.get_param
        self.use_map_odom = gp('~use_map_odom', True)
        local_topic = gp('~local_odom_topic', '/odometry/local')
        map_odom_topic = gp('~map_odom_topic', '/odometry/filtered')
        if rospy.resolve_name(local_topic) == rospy.resolve_name(map_odom_topic):
            rospy.logwarn("local and map odometry are the same topic (%s): race line "
                          "will use our own cone map in the local frame" % local_topic)
            self.use_map_odom = False
            self.planner.p['race_map_source'] = 'own'

        self.local_hist = deque(maxlen=600)     # (stamp, pose)
        self.local_pose = None
        self.local_frame = 'odom'
        self.markers_time = None
        self.warned_no_sub = False
        self.last_race_marker = 0.0

        rospy.Subscriber(local_topic, Odometry, self._local_cb, queue_size=10)
        if self.use_map_odom:
            rospy.Subscriber(map_odom_topic, Odometry, self._map_odom_cb, queue_size=10)
            rospy.Subscriber(gp('~map_markers_topic', '/cone_map_markers'), MarkerArray,
                             self._markers_cb, queue_size=1)
            rospy.Subscriber(gp('~map_topic', '/cone_map'), PoseArray, self._map_cb,
                             queue_size=1)
        rospy.Subscriber(gp('~coloured_topic', '/cone_detections_fused_coloured'),
                         ConeDetectionArray, self._coloured_cb, queue_size=1)
        rospy.Subscriber(gp('~plain_topic', '/cone_detections_fused'), PoseArray,
                         self._plain_cb, queue_size=1)

        self.traj_pub = rospy.Publisher(gp('~trajectory_topic', '/qcar/trajectory_topic'),
                                        TrajectoryMessage, queue_size=1)
        self.status_pub = rospy.Publisher('/guidance_status', String, queue_size=1)
        self.race_pub = rospy.Publisher('/guidance_race_line', Marker, queue_size=1, latch=True)

        rospy.loginfo("online_guidance_node v5 started: local=%s map_odom=%s race=%s "
                      "total_laps=%d" % (local_topic, map_odom_topic if self.use_map_odom
                                         else 'off', self.p['race_enabled'],
                                         self.p['total_laps']))

    # ------------------------------------------------------------ logging
    @staticmethod
    def _log(level, msg):
        if level == 'warn':
            rospy.logwarn(msg)
        elif level == 'debug':
            rospy.logdebug(msg)
        else:
            rospy.loginfo(msg)

    # ------------------------------------------------------------ helpers
    def _stamp(self, header):
        t = header.stamp.to_sec() if header is not None else 0.0
        return t if t > 0 else rospy.get_time()

    def _local_at(self, stamp):
        if not self.local_hist:
            return self.local_pose
        return min(self.local_hist, key=lambda h: abs(h[0] - stamp))[1]

    # ------------------------------------------------------------ callbacks
    def _local_cb(self, msg):
        pose = pose_of(msg)
        with self.lock:
            self.local_pose = pose
            self.local_hist.append((self._stamp(msg.header), pose))
            if msg.header.frame_id:
                self.local_frame = msg.header.frame_id

    def _map_odom_cb(self, msg):
        if self.local_pose is None:
            return
        stamp = self._stamp(msg.header)
        with self.lock:
            self.planner.on_slam_pose(pose_of(msg), self._local_at(stamp), rospy.get_time())

    def _coloured_cb(self, msg):
        if self.local_pose is None:
            return
        dets = [(d.position.x, d.position.y, d.colour) for d in msg.detections]
        with self.lock:
            self.planner.on_detections(dets, self._local_at(self._stamp(msg.header)),
                                       rospy.get_time())

    def _plain_cb(self, msg):
        if self.local_pose is None:
            return
        dets = [(q.position.x, q.position.y, gc.UNKNOWN) for q in msg.poses]
        with self.lock:
            self.planner.on_detections(dets, self._local_at(self._stamp(msg.header)),
                                       rospy.get_time())

    def _markers_cb(self, msg):
        cones = [(m.pose.position.x, m.pose.position.y, marker_colour(m))
                 for m in msg.markers
                 if m.ns == 'cones' and m.action == Marker.ADD and m.type == Marker.SPHERE]
        if not cones:
            return
        with self.lock:
            self.markers_time = rospy.get_time()
            self.planner.on_map(cones, rospy.get_time())

    def _map_cb(self, msg):
        now = rospy.get_time()
        # only if the coloured markers are not arriving
        if self.markers_time is not None and now - self.markers_time < 5.0:
            return
        cones = [(q.position.x, q.position.y, gc.UNKNOWN) for q in msg.poses]
        with self.lock:
            self.planner.on_map(cones, now)

    # ------------------------------------------------------------ publish
    def _publish_traj(self, t):
        msg = TrajectoryMessage()
        msg.waypoint_times = t['t']
        msg.waypoint_x = t['x']
        msg.waypoint_y = t['y']
        msg.velocity = t['velocity']
        self.traj_pub.publish(msg)

    def _publish_race_line(self):
        pts = self.planner.race_line_local()
        if pts is None:
            return
        m = Marker()
        m.header.frame_id = self.local_frame
        m.header.stamp = rospy.Time.now()
        m.ns, m.id = 'race_line', 0
        m.type, m.action = Marker.LINE_STRIP, Marker.ADD
        m.scale.x = 0.04
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.2, 0.8, 1.0
        m.pose.orientation.w = 1.0
        for x, y in list(zip(pts[0], pts[1])) + [(pts[0][0], pts[1][0])]:
            m.points.append(Point(x=float(x), y=float(y), z=0.03))
        self.race_pub.publish(m)

    def tick(self):
        """One planning cycle (called at plan_rate_hz by spin())."""
        now = rospy.get_time()
        with self.lock:
            pose = self.local_pose
            out = None if pose is None else self.planner.step(now, pose)
        if out is None:
            rospy.loginfo_throttle(2, "waiting for local odometry...")
            return None

        if self.traj_pub.get_num_connections() == 0:
            rospy.logwarn_throttle(5, "nothing subscribed to the trajectory topic -- "
                                      "controller not running or exited")
            self.warned_no_sub = True
        elif self.warned_no_sub:
            rospy.loginfo("controller connected")
            self.warned_no_sub = False

        if out['traj'] is not None:
            self._publish_traj(out['traj'])
        st = out['status']
        self.status_pub.publish(String(data=json.dumps(st)))
        rospy.loginfo_throttle(2, "[%s/%s] lap %d | v %s | mem %d%s" % (
            st['phase'], st['mode'], st['lap'],
            'hold' if st['cmd_velocity'] is None else '%.2f' % st['cmd_velocity'],
            st['mem'], '' if st.get('deviation') is None
            else ' | dev %.2f m' % st['deviation']))
        if self.planner.race is not None and now - self.last_race_marker > 2.0:
            self._publish_race_line()
            self.last_race_marker = now
        return out

    def spin(self):
        rate = rospy.Rate(self.p['plan_rate_hz'])
        while not rospy.is_shutdown():
            self.tick()
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('online_guidance_node')
    GuidanceNode().spin()
