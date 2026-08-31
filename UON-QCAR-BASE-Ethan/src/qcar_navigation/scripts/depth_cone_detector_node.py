#!/usr/bin/env python3

import math
import rospy
import numpy as np

from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from geometry_msgs.msg import PoseArray, Pose

class DepthConeDetectorNode:
    def __init__(self):
        rospy.init_node("depth_cone_detector_node")

        self.cloud_topic = rospy.get_param("~cloud_topic", "/depth_camera/points")
        self.output_topic = rospy.get_param("~output_topic", "/cone_detections_depth")

        self.min_x = rospy.get_param("~min_x", 0.2)
        self.max_x = rospy.get_param("~max_x", 3.5)
        self.max_abs_y = rospy.get_param("~max_abs_y", 3.0)

        self.min_z = rospy.get_param("~min_z", 0.02)
        self.max_z = rospy.get_param("~max_z", 0.25)

        self.cluster_tolerance = rospy.get_param("~cluster_tolerance", 0.1)
        self.min_cluster_size = rospy.get_param("~min_cluster_size", 12)
        self.max_cluster_size = rospy.get_param("~max_cluster_size", 250)

        self.min_cluster_width = rospy.get_param("~min_cluster_width", 0.04)
        self.max_cluster_width = rospy.get_param("~max_cluster_width", 0.2)

        self.pub = rospy.Publisher(self.output_topic, PoseArray, queue_size=1)
        rospy.Subscriber(self.cloud_topic, PointCloud2, self.cloud_callback, queue_size=1)

        rospy.loginfo("depth_cone_detector_node started")

    def cloud_callback(self, msg):
        raw_points = []

        for p in point_cloud2.read_points(msg, field_names=("x","y","z"), skip_nans=True):
            x, y, z = p

            if x < self.min_x or x> self.max_x:
                continue
            if abs(y) > self.max_abs_y:
                continue
            if z < self.min_z or z > self.max_z:
                continue

            raw_points.append([x, y, z])

        if not raw_points:
            self.publish_pose_array([], msg.header.stamp)
            return

        points = np.array(raw_points, dtype=float)

        xy = points[:, :2]

        clusters = self.euclidean_clusters(xy, self.cluster_tolerance)

        detections = []

        for cluster_idx in clusters:
            if len(cluster_idx) < self.min_cluster_size or len(cluster_idx) > self.max_cluster_size:
                continue 

            cluster_pts = points[cluster_idx]
            cluster_xy = cluster_pts[:, :2]

            min_xyz = np.min(cluster_pts, axis=0)
            max_xyz = np.max(cluster_pts, axis=0)

            height = float(max_xyz[2] - min_xyz[2])

            if height < 0.03 or height > 0.35:
                continue

            min_xy = np.min(cluster_xy, axis=0)
            max_xy = np.max(cluster_xy, axis=0)
            size_xy = max_xy - min_xy
            width = float(np.linalg.norm(size_xy))

            if width < self.min_cluster_width or width > self.max_cluster_width:
                continue

            center = np.mean(cluster_xy, axis=0)
            detections.append(center)

        self.publish_pose_array(detections, msg.header.stamp)

    def euclidean_clusters(self, xy_points, tol):
        n = len(xy_points)
        visited = np.zeros(n, dtype=bool)
        cluster = []
        clusters = []

        for i in range(n):
            if visited[i]:
                continue
            

            queue = [i]
            visited[i] = True
            cluster = []

            while queue:
                idx = queue.pop()
                cluster.append(idx)

                dists = np.linalg.norm(xy_points - xy_points[idx], axis=1)
                neighbors = np.where((dists <= tol) & (~visited))[0]

                for nb in neighbors:
                    visited[nb] = True
                    queue.append(nb)

            clusters.append(cluster)

        return clusters

    def publish_pose_array(self, detections, stamp):
        msg = PoseArray()
        msg.header.stamp = stamp if stamp != rospy.Time() else rospy.Time.now()
        msg.header.frame_id = "base_footprint"

        for center in detections:
            pose = Pose()
            pose.position.x = float(center[0])
            pose.position.y = float(center[1])
            pose.position.z = 0.0
            pose.orientation.w = 1.0
            msg.poses.append(pose)

        self.pub.publish(msg)


if __name__ == "__main__":
    try:
        node = DepthConeDetectorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass