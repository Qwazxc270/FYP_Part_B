#!/usr/bin/env python3

import math
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, Pose


class GazeboConeDetectorNode:
    def __init__(self):
        rospy.init_node("gazebo_cone_detector_node")

        self.odom_topic = rospy.get_param("~odom_topic", "/odometry/filtered")
        self.output_topic = rospy.get_param("~output_topic", "/cone_detections_relative")
        self.max_range = rospy.get_param("~max_range", 6.0)
        self.fov_deg = rospy.get_param("~fov_deg", 120.0)

        self.cones_world = [
            # blue cones
            (-5.0, 4.0), (-6.03528, 3.8637), (-7.0, 3.4641), (-7.82843, 2.82843),
            (-8.4641, 2.0), (-8.8637, 1.03528), (-9.0, 0.0), (-8.8637, -1.03528),
            (-8.4641, -2.0), (-7.82843, -2.82843), (-7.0, -3.4641), (-6.03528, -3.8637),
            (-5.0, -4.0), (-4.0, -4.0), (-3.0, -4.0), (-2.0, -4.0),
            (-1.0, -4.0), (0.0, -4.0), (1.0, -4.0), (2.03528, -3.8637),
            (3.0, -3.4641), (3.82843, -2.82843), (4.4641, -2.0), (4.8637, -1.03528),
            (5.0, 0.0), (4.8637, 1.03528), (4.4641, 2.0), (3.82843, 2.82843),
            (3.0, 3.4641), (2.03528, 3.8637), (1.0, 4.0), (0.0, 4.0),
            (-1.0, 4.0), (-2.0, 4.0), (-3.0, 4.0), (-4.0, 4.0),

            # yellow cones
            (-5.0, 2.0), (-5.51764, 1.93185), (-6.0, 1.73205), (-6.41421, 1.41421),
            (-6.73205, 1.0), (-6.93185, 0.517638), (-7.0, 0.0), (-6.93185, -0.517638),
            (-6.73205, -1.0), (-6.41421, -1.41421), (-6.0, -1.73205), (-5.51764, -1.93185),
            (-5.0, -2.0), (-4.0, -2.0), (-3.0, -2.0), (-2.0, -2.0),
            (-1.0, -2.0), (0.0, -2.0), (1.0, -2.0), (1.51764, -1.93185),
            (2.0, -1.73205), (2.41421, -1.41421), (2.73205, -1.0), (2.93185, -0.517638),
            (3.0, 0.0), (2.93185, 0.517638), (2.73205, 1.0), (2.41421, 1.41421),
            (2.0, 1.73205), (1.51764, 1.93185), (1.0, 2.0), (0.0, 2.0),
            (-1.0, 2.0), (-2.0, 2.0), (-3.0, 2.0), (-4.0, 2.0),

            # orange cones
            (-5.2, 4.3), (-5.2, 1.7), (-4.85, 4.3), (-4.85, 1.7),
        ]
        
        self.pose_ready = False
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=1)
        self.pub = rospy.Publisher(self.output_topic, PoseArray, queue_size=1)

        self.timer = rospy.Timer(rospy.Duration(0.1), self.publish_detections)

        rospy.loginfo("gazebo_cone_detector_node started")

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2.0*(q.w*q.z + q.x*q.y)
        cosy_cosp = 1.0 - 2.0*(q.y*q.y + q.z*q.z)
        self.yaw = np.arctan2(siny_cosp, cosy_cosp)

        self.pose_ready = True

    def publish_detections(self, event):
        if not self.pose_ready:
            return  

        msg = PoseArray()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_footprint"

        c = np.cos(self.yaw)
        s = np.sin(self.yaw)

        R_bw = np.array([
            [c, s],
            [-s, c]
        ])

        fov_half = math.radians(self.fov_deg)/2.0

        for cx, cy in self.cones_world:
            dx = cx - self.x
            dy = cy - self.y

            rel = R_bw@np.array([[dx], [dy]])
            bx = rel[0, 0]
            by = rel[1, 0]

            rng = np.hypot(bx, by)
            bearing = np.arctan2(by, bx)

            if rng > self.max_range:
                continue

            if abs(bearing) > fov_half:
                continue

            pose = Pose()
            pose.position.x = bx
            pose.position.y = by
            pose.position.z = 0.0
            pose.orientation.w = 1.0
            msg.poses.append(pose)

        self.pub.publish(msg)

if __name__ == "__main__":
    try:
        node = GazeboConeDetectorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
