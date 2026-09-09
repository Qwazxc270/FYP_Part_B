#!/usr/bin/env python3

import math
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray, Pose

class LidarConeDetectorNode:
    def __init__(self):
        rospy.init_node("lidar_cone_detector_node")

        self.scan_topic = rospy.get_param("~scan_topic", "/scan")
        self.output_topic = rospy.get_param("~output_topic", "/cone_detections_lidar")

        self.min_range = rospy.get_param("~min_range", 0.1)
        self.max_range = rospy.get_param("~max_range", 6.0)

        self.cluster_distance_threshold = rospy.get_param("~cluster_distance_threshold", 0.18)

        self.min_cluster_points = rospy.get_param("~min_cluster_points", 2)
        self.max_cluster_points = rospy.get_param("~max_cluster_points", 12)

        self.min_cluster_width = rospy.get_param("~min_cluster_width", 0.03)
        self.max_cluster_width = rospy.get_param("~max_cluster_width", 0.3)

        self.scan_msg = None

        rospy.Subscriber(self.scan_topic, LaserScan, self.scan_callback, queue_size=1)
        self.pub = rospy.Publisher(self.output_topic, PoseArray, queue_size=1)

    def scan_callback(self, msg):
        self.scan_msg = msg
        self.publish_detections()

    def polar_to_xy(self, r, theta):
        x = r*np.cos(theta)
        y = r*np.sin(theta)
        return np.array([x, y], dtype=float)

    def publish_detections(self):
        if self.scan_msg is None:
            return

        scan = self.scan_msg
        points = []

        angle = scan.angle_min
        for r in scan.ranges:
            if math.isfinite(r) and self.min_range <= r <= self.max_range:
                xy = self.polar_to_xy(r, angle)
                points.append({
                    "xy": xy,
                    "range": r,
                    "angle": angle 
                })
            else:
                points.append(None)
            angle += scan.angle_increment

        clusters = []
        current_cluster = []

        for p in points:
            if p is None:
                if current_cluster:
                    clusters.append(current_cluster)
                    current_cluster = []
                continue
            if not current_cluster:
                current_cluster = [p]
                continue

            prev = current_cluster[-1]
            dist = np.linalg.norm(p["xy"] - prev["xy"])

            if dist <= self.cluster_distance_threshold:
                current_cluster.append(p)
            else:
                clusters.append(current_cluster)
                current_cluster = [p]

        if current_cluster:
            clusters.append(current_cluster)

        pose_array = PoseArray()
        pose_array.header.stamp = scan.header.stamp if scan.header.stamp != rospy.Time() else rospy.Time.now()
        pose_array.header.frame_id = "base_footprint"

        for cluster in clusters:
            n = len(cluster)
            if n < self.min_cluster_points or n > self.max_cluster_points:
                continue

            start_xy = cluster[0]["xy"]
            end_xy = cluster[-1]["xy"]

            cluster_width = np.linalg.norm(end_xy - start_xy)

            if cluster_width < self.min_cluster_width or cluster_width > self.max_cluster_width:
                continue
            
            xy_points = np.array([p["xy"] for p in cluster])
            center = np.mean(xy_points, axis=0)

            pose = Pose()
            pose.position.x = float(center[0])
            pose.position.y = float(center[1])
            pose.position.z = 0.0
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)

        self.pub.publish(pose_array)

if __name__ == "__main__":
    try:
        node = LidarConeDetectorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass