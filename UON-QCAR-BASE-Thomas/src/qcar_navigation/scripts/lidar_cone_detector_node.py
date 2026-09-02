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

        # Adaptive clustering
        self.base_cluster_distance = rospy.get_param("~base_cluster_distance", 0.04)
        self.range_scale_cluster_distance = rospy.get_param("~range_scale_cluster_distance", 0.03)
        self.max_cluster_distance = rospy.get_param("~max_cluster_distance", 0.18)

        # Cluster size
        self.min_cluster_points = rospy.get_param("~min_cluster_points", 3)
        self.max_cluster_points = rospy.get_param("~max_cluster_points", 16)

        # Geometric checks
        self.min_cluster_width = rospy.get_param("~min_cluster_width", 0.04)
        self.max_cluster_width = rospy.get_param("~max_cluster_width", 0.35)
        self.max_cluster_depth = rospy.get_param("~max_cluster_depth", 0.20)
        self.max_cluster_radius_span = rospy.get_param("~max_cluster_radius_span", 0.18)
        self.max_cluster_angle_span = rospy.get_param("~max_cluster_angle_span", 0.14)
        self.max_linearity_ratio = rospy.get_param("~max_linearity_ratio", 10.0)
        self.min_cluster_density = rospy.get_param("~min_cluster_density", 6.0)
        self.max_cluster_mean_range = rospy.get_param("~max_cluster_mean_range", 5.5)

        # Cone face -> cone center correction
        self.cone_radius = rospy.get_param("~cone_radius", 0.08)

        self.debug = rospy.get_param("~debug", False)

        self.scan_msg = None

        rospy.Subscriber(self.scan_topic, LaserScan, self.scan_callback, queue_size=1)
        self.pub = rospy.Publisher(self.output_topic, PoseArray, queue_size=1)

        rospy.loginfo("lidar_cone_detector_node started")

    def scan_callback(self, msg):
        self.scan_msg = msg
        self.publish_detections()

    def polar_to_xy(self, r, theta):
        return np.array([r * np.cos(theta), r * np.sin(theta)], dtype=float)

    def adaptive_cluster_threshold(self, r1, r2):
        r_mean = 0.5 * (r1 + r2)
        thresh = self.base_cluster_distance + self.range_scale_cluster_distance * r_mean
        return min(thresh, self.max_cluster_distance)

    def build_points(self, scan):
        points = []
        angle = scan.angle_min

        for i, r in enumerate(scan.ranges):
            if math.isfinite(r) and self.min_range <= r <= self.max_range:
                xy = self.polar_to_xy(r, angle)
                points.append({
                    "idx": i,
                    "xy": xy,
                    "range": float(r),
                    "angle": float(angle),
                })
            else:
                points.append(None)
            angle += scan.angle_increment

        return points

    def build_clusters(self, points):
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
            local_thresh = self.adaptive_cluster_threshold(prev["range"], p["range"])

            if dist <= local_thresh:
                current_cluster.append(p)
            else:
                clusters.append(current_cluster)
                current_cluster = [p]

        if current_cluster:
            clusters.append(current_cluster)

        return clusters

    def cluster_stats(self, cluster):
        xy_points = np.array([p["xy"] for p in cluster], dtype=float)
        ranges = np.array([p["range"] for p in cluster], dtype=float)
        angles = np.array([p["angle"] for p in cluster], dtype=float)

        center_mean = np.mean(xy_points, axis=0)
        start_xy = xy_points[0]
        end_xy = xy_points[-1]

        width = float(np.linalg.norm(end_xy - start_xy))
        mean_range = float(np.mean(ranges))
        radius_span = float(np.max(ranges) - np.min(ranges))
        angle_span = float(np.max(angles) - np.min(angles))

        centered = xy_points - center_mean
        if len(cluster) > 1:
            cov = centered.T @ centered / (len(cluster) - 1)
        else:
            cov = np.eye(2) * 1e-9

        eigvals, eigvecs = np.linalg.eigh(cov)
        eigvals = np.maximum(np.sort(eigvals), 1e-9)

        major_axis = eigvecs[:, np.argmax(eigvals)]
        minor_axis = eigvecs[:, np.argmin(eigvals)]

        projected_minor = centered @ minor_axis
        minor_extent = float(np.max(projected_minor) - np.min(projected_minor))

        linearity_ratio = float(eigvals[1] / eigvals[0])
        density = float(len(cluster) / max(width, 1e-6))

        return {
            "xy_points": xy_points,
            "mean_range": mean_range,
            "radius_span": radius_span,
            "angle_span": angle_span,
            "width": width,
            "depth": minor_extent,
            "linearity_ratio": linearity_ratio,
            "density": density,
        }

    def cluster_is_valid(self, cluster):
        n = len(cluster)
        if n < self.min_cluster_points or n > self.max_cluster_points:
            return False

        stats = self.cluster_stats(cluster)

        if stats["width"] < self.min_cluster_width or stats["width"] > self.max_cluster_width:
            return False

        if stats["depth"] > self.max_cluster_depth:
            return False

        if stats["mean_range"] > self.max_cluster_mean_range:
            return False

        if stats["radius_span"] > self.max_cluster_radius_span:
            return False

        if stats["angle_span"] > self.max_cluster_angle_span:
            return False

        if stats["linearity_ratio"] > self.max_linearity_ratio:
            return False

        if stats["density"] < self.min_cluster_density:
            return False

        return True

    def estimate_cone_position(self, cluster):
        # More stable than centroid for partial cone surfaces
        mid_idx = len(cluster) // 2
        mid_xy = cluster[mid_idx]["xy"]
        mid_norm = np.linalg.norm(mid_xy)

        if mid_norm > 1e-6:
            radial_dir = mid_xy / mid_norm
            center = mid_xy + self.cone_radius * radial_dir
        else:
            center = mid_xy.copy()

        return center

    def publish_detections(self):
        if self.scan_msg is None:
            return

        scan = self.scan_msg
        points = self.build_points(scan)
        clusters = self.build_clusters(points)

        pose_array = PoseArray()
        pose_array.header.stamp = scan.header.stamp if scan.header.stamp != rospy.Time() else rospy.Time.now()
        pose_array.header.frame_id = scan.header.frame_id

        raw_count = len(clusters)
        valid_count = 0

        for cluster in clusters:
            if not self.cluster_is_valid(cluster):
                continue

            center = self.estimate_cone_position(cluster)

            pose = Pose()
            pose.position.x = float(center[0])
            pose.position.y = float(center[1])
            pose.position.z = 0.0
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)
            valid_count += 1

        if self.debug:
            rospy.loginfo_throttle(
                1.0,
                "raw_clusters=%d valid_clusters=%d detections=%d",
                raw_count,
                valid_count,
                len(pose_array.poses),
            )

        self.pub.publish(pose_array)


if __name__ == "__main__":
    try:
        node = LidarConeDetectorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass