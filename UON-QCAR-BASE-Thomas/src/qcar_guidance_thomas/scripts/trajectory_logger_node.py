#! /usr/bin/env python3
"""
Trajectory logger -- records odometry + trajectory stats to CSV.

Output lands in ~/guidance_log_<timestamp>.csv with columns:
    t, car_x, car_y, n_waypoints, mean_velocity

Use this data for report plots: driven path vs planned path, speed over
time, etc. (same spirit as the Part A CTE analysis in MATLAB).
"""

import rospy
import csv
import os
from qcar_guidance.msg import TrajectoryMessage
from nav_msgs.msg import Odometry


class Logger:
    def __init__(self):
        path = os.path.expanduser(
            "~/guidance_log_%d.csv" % int(rospy.get_time()))
        self.f = open(path, "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow(["t", "car_x", "car_y", "n_waypoints", "mean_velocity"])
        self.last_traj = None
        rospy.loginfo("trajectory_logger_node logging to %s", path)

    def traj_cb(self, msg):
        self.last_traj = msg

    def odom_cb(self, msg):
        n = len(self.last_traj.waypoint_x) if self.last_traj else 0
        v = self.last_traj.velocity if self.last_traj else 0.0
        self.w.writerow([rospy.get_time(),
                         msg.pose.pose.position.x,
                         msg.pose.pose.position.y, n, v])


if __name__ == '__main__':
    rospy.init_node('trajectory_logger_node')
    lg = Logger()
    rospy.Subscriber('/qcar/trajectory_topic', TrajectoryMessage, lg.traj_cb)
    rospy.Subscriber('/odometry/filtered', Odometry, lg.odom_cb)
    rospy.on_shutdown(lambda: lg.f.close())
    rospy.spin()
