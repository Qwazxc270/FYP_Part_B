#!/usr/bin/env python3

import rospy
import numpy as np
import tf
import tf.transformations as tft
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from geometry_msgs.msg import PoseArray, Pose


class DepthConeDetectorNode:
    def __init__(self):
        rospy.init_node("depth_cone_detector_node")

        self.cloud_topic = rospy.get_param("~cloud_topic", "/depth_camera/points")
        self.output_topic = rospy.get_param("~output_topic", "/cone_detections_depth")
        self.target_frame = rospy.get_param("~target_frame", "base_footprint")

        # Strong crop in base frame
        self.min_x = rospy.get_param("~min_x", 0.3)
        self.max_x = rospy.get_param("~max_x", 4.0)
        self.max_abs_y = rospy.get_param("~max_abs_y", 2.5)
        self.min_z = rospy.get_param("~min_z", 0.03)
        self.max_z = rospy.get_param("~max_z", 0.50)

        # Fast coarse grouping
        self.grid_size = rospy.get_param("~grid_size", 0.20)
        self.min_points_per_cell = rospy.get_param("~min_points_per_cell", 4)

        # Downsample input
        self.point_stride = rospy.get_param("~point_stride", 8)

        self.debug = rospy.get_param("~debug", True)

        self.tf_listener = tf.TransformListener()

        self.pub = rospy.Publisher(self.output_topic, PoseArray, queue_size=1)
        rospy.Subscriber(self.cloud_topic, PointCloud2, self.cloud_callback, queue_size=1)

        rospy.loginfo("depth_cone_detector_node started")

    def cloud_callback(self, msg):
        source_frame = msg.header.frame_id

        raw_points = []
        total_points = 0

        for i, p in enumerate(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)):
            total_points += 1

            if (i % self.point_stride) != 0:
                continue

            raw_points.append([p[0], p[1], p[2]])

        if not raw_points:
            self.publish_pose_array([], msg.header.stamp)
            return

        pts_cam = np.array(raw_points, dtype=float)

        if not source_frame:
            # sensor not fully initialised yet (empty frame_id) -- skip
            return


        try:
            self.tf_listener.waitForTransform(
                self.target_frame,
                source_frame,
                rospy.Time(0),
                rospy.Duration(0.1)
            )
            trans, rot = self.tf_listener.lookupTransform(
                self.target_frame,
                source_frame,
                rospy.Time(0)
            )
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn_throttle(1.0, "Depth detector TF lookup failed: %s -> %s", source_frame, self.target_frame)
            self.publish_pose_array([], msg.header.stamp)
            return

        T = tft.quaternion_matrix(rot)
        T[0:3, 3] = np.array(trans)

        pts_h = np.hstack([pts_cam, np.ones((pts_cam.shape[0], 1))])
        pts_base = (T @ pts_h.T).T[:, :3]

        mask = (
            (pts_base[:, 0] >= self.min_x) &
            (pts_base[:, 0] <= self.max_x) &
            (np.abs(pts_base[:, 1]) <= self.max_abs_y) &
            (pts_base[:, 2] >= self.min_z) &
            (pts_base[:, 2] <= self.max_z)
        )
        pts_base = pts_base[mask]

        if self.debug:
            rospy.loginfo_throttle(
                1.0,
                "depth fast: total=%d sampled=%d filtered=%d",
                total_points,
                len(raw_points),
                pts_base.shape[0],
            )

        if pts_base.shape[0] == 0:
            self.publish_pose_array([], msg.header.stamp)
            return

        # Simple grid-based grouping instead of expensive clustering
        xy = pts_base[:, :2]
        cell_dict = {}

        for pt in xy:
            cx = int(np.floor(pt[0] / self.grid_size))
            cy = int(np.floor(pt[1] / self.grid_size))
            key = (cx, cy)
            if key not in cell_dict:
                cell_dict[key] = []
            cell_dict[key].append(pt)

        detections = []
        for pts in cell_dict.values():
            if len(pts) < self.min_points_per_cell:
                continue
            pts_arr = np.array(pts, dtype=float)
            center = np.mean(pts_arr, axis=0)
            detections.append(center)

        if self.debug:
            rospy.loginfo_throttle(
                1.0,
                "depth fast: cells=%d detections=%d",
                len(cell_dict),
                len(detections),
            )

        self.publish_pose_array(detections, msg.header.stamp)

    def publish_pose_array(self, detections, stamp):
        msg = PoseArray()
        msg.header.stamp = stamp if stamp != rospy.Time() else rospy.Time.now()
        msg.header.frame_id = self.target_frame

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