#! /usr/bin/env python3
"""
trajectory_logger_node -- records a run to CSV for report plots.

Output: ~/guidance_log_<unix time>.csv, one row per odometry message:
  t, x, y, yaw, speed, phase, mode, lap, deviation, cmd_velocity, n_waypoints

phase = MAPPING / RACE / FINISHED, mode = CREEP / TRACK / EDGE / RACE /
RECOVERY / LOST / STOP, deviation = distance from the race line (race laps).
"""
import csv
import json
import math
import os
import threading
import time

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from qcar_guidance.msg import TrajectoryMessage


class Logger:
    def __init__(self):
        topic = rospy.get_param('~odom_topic', '/odometry/local')
        self.path = os.path.expanduser('~/guidance_log_%d.csv' % int(time.time()))
        self.f = open(self.path, 'w', newline='')
        self.w = csv.writer(self.f)
        self.w.writerow(['t', 'x', 'y', 'yaw', 'speed', 'phase', 'mode', 'lap',
                         'deviation', 'cmd_velocity', 'n_waypoints'])
        self.status = {}
        self.n_wp = 0
        self.lock = threading.Lock()
        self.closed = False
        rospy.Subscriber(topic, Odometry, self.odom_cb, queue_size=50)
        rospy.Subscriber('/guidance_status', String, self.status_cb, queue_size=5)
        rospy.Subscriber('/qcar/trajectory_topic', TrajectoryMessage, self.traj_cb, queue_size=5)
        rospy.on_shutdown(self.close)
        rospy.loginfo("trajectory_logger_node logging %s to %s" % (topic, self.path))

    def status_cb(self, msg):
        try:
            self.status = json.loads(msg.data)
        except ValueError:
            pass

    def traj_cb(self, msg):
        self.n_wp = len(msg.waypoint_x)

    def odom_cb(self, msg):
        p = msg.pose.pose
        q = p.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        v = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        s = self.status
        with self.lock:
            if self.closed:
                return
            self.w.writerow([rospy.get_time(), p.position.x, p.position.y, yaw, v,
                             s.get('phase', ''), s.get('mode', ''), s.get('lap', ''),
                             s.get('deviation', ''), s.get('cmd_velocity', ''), self.n_wp])

    def close(self):
        with self.lock:
            if not self.closed:
                self.closed = True
                self.f.close()


if __name__ == '__main__':
    rospy.init_node('trajectory_logger_node')
    Logger()
    rospy.spin()
