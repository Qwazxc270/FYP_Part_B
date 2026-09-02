#!/usr/bin/env python3

import rospy
import numpy as np
import tf
import tf.transformations as tft
import message_filters
from std_msgs.msg import Header
from qcar_navigation.msg import ConeDetection, ConeDetectionArray
from geometry_msgs.msg import PoseArray, Pose


class ConeFusionNode:
    def __init__(self):
        rospy.init_node("cone_fusion_node")

        self.lidar_topic = rospy.get_param("~lidar_topic", "/cone_detections_lidar")
        self.depth_topic = rospy.get_param("~depth_topic", "/cone_detections_depth")
        self.output_topic = rospy.get_param("~output_topic", "/cone_detections_fused")
        self.target_frame = rospy.get_param("~target_frame", "base_footprint")
        self.match_distance = rospy.get_param("~match_distance", 0.3)
        self.sync_slop = rospy.get_param("~sync_slop", 0.15)
        self.debug = rospy.get_param("~debug", True)

        self.camera_topic = rospy.get_param("~camera_topic", "/cone_detections_camera")
        self.coloured_output_topic = rospy.get_param("~coloured_output_topic", "/cone_detections_fused_coloured")
        self.colour_match_distance = rospy.get_param("~colour_match_distance", 0.35)
        self.latest_lidar_points = []
        self.tf_listener = tf.TransformListener()

        self.pub = rospy.Publisher(self.output_topic, PoseArray, queue_size=1)
        self.pub_coloured = rospy.Publisher(self.coloured_output_topic, ConeDetectionArray, queue_size=1)

        # Use ApproximateTimeSynchronizer for temporal alignment
        lidar_sub = message_filters.Subscriber(self.lidar_topic, PoseArray)
        depth_sub = message_filters.Subscriber(self.depth_topic, PoseArray)

        self.sync = message_filters.ApproximateTimeSynchronizer(
            [lidar_sub, depth_sub],
            queue_size=5,
            slop=self.sync_slop,
        )
        self.sync.registerCallback(self.synced_callback)

        # Fallback: also accept lidar-only or depth-only when the other
        # sensor has not published for a while
        self.lidar_msg = None
        self.depth_msg = None
        self.lidar_stamp = rospy.Time(0)
        self.depth_stamp = rospy.Time(0)

        rospy.Subscriber(self.lidar_topic, PoseArray, self.lidar_fallback_cb, queue_size=1)
        rospy.Subscriber(self.depth_topic, PoseArray, self.depth_fallback_cb, queue_size=1)
        rospy.Subscriber(self.camera_topic, ConeDetectionArray, self._camera_coloured_cb, queue_size=1)
        rospy.Subscriber(self.lidar_topic, PoseArray, self._lidar_points_cb, queue_size=1)

        self.fallback_timeout = rospy.get_param("~fallback_timeout", 0.5)
        rospy.Timer(rospy.Duration(0.1), self.fallback_publish)

        rospy.loginfo("cone_fusion_node started (synced + fallback)")

    # ------------------------------------------------------------------
    # Fallback individual callbacks (used when one sensor is missing)
    # ------------------------------------------------------------------
    def lidar_fallback_cb(self, msg):
        self.lidar_msg = msg
        self.lidar_stamp = rospy.Time.now()

    def depth_fallback_cb(self, msg):
        self.depth_msg = msg
        self.depth_stamp = rospy.Time.now()

    def _lidar_points_cb(self, msg):
        self.latest_lidar_points = self.transform_poses_to_frame(msg, self.target_frame)

    def _camera_coloured_cb(self, msg):
        lidar_pts = list(self.latest_lidar_points)
        used = set()
        out_dets = []

        for det in msg.detections:
            cam_pt = np.array([det.position.x, det.position.y], dtype=float)

            best_j, best_d = -1, 1e9
            for j, lp in enumerate(lidar_pts):
                if j in used:
                    continue
                d = np.linalg.norm(cam_pt - lp)
                if d < best_d:
                    best_d, best_j = d, j

            if best_j >= 0 and best_d <= self.colour_match_distance:
                fused_pt = 0.5*(cam_pt + lidar_pts[best_j])
                used.add(best_j)
            else:
                fused_pt = cam_pt

            new_det = ConeDetection()
            new_det.header = det.header
            new_det.colour = det.colour
            new_det.position.x = float(fused_pt[0])
            new_det.position.y = float(fused_pt[1])
            new_det.position.z = 0.0
            new_det.range = float(np.linalg.norm(fused_pt))
            new_det.position_covariance = det.position_covariance
            out_dets.append(new_det)
        out = ConeDetectionArray()
        out.header = Header(stamp=msg.header.stamp, frame_id=self.target_frame)
        out.detections = out_dets
        self.pub_coloured.publish(out)

        if self.debug:
            rospy.loginfo_throttle(1.0, "fusion coloured: camera=%d lidar=%d matched=%d", len(msg.detections), len(lidar_pts), len(used))


    # ------------------------------------------------------------------
    # TF helpers
    # ------------------------------------------------------------------
    def transform_poses_to_frame(self, pose_array, target_frame):
        """Transform a PoseArray into target_frame using TF."""
        source_frame = pose_array.header.frame_id
        if not source_frame:
            source_frame = "base_footprint"

        if source_frame == target_frame:
            return self.posearray_to_points(pose_array)

        try:
            self.tf_listener.waitForTransform(
                target_frame, source_frame, rospy.Time(0), rospy.Duration(0.1)
            )
            trans, rot = self.tf_listener.lookupTransform(
                target_frame, source_frame, rospy.Time(0)
            )
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn_throttle(
                2.0, "Fusion TF lookup failed %s -> %s: %s", source_frame, target_frame, str(e)
            )
            return self.posearray_to_points(pose_array)

        T = tft.quaternion_matrix(rot)
        T[0:3, 3] = np.array(trans)

        pts = []
        for p in pose_array.poses:
            pt_h = np.array([p.position.x, p.position.y, p.position.z, 1.0])
            pt_tf = T @ pt_h
            pts.append(np.array([pt_tf[0], pt_tf[1]], dtype=float))
        return pts

    def posearray_to_points(self, msg):
        pts = []
        if msg is None:
            return pts
        for p in msg.poses:
            pts.append(np.array([p.position.x, p.position.y], dtype=float))
        return pts

    # ------------------------------------------------------------------
    # Core fusion logic
    # ------------------------------------------------------------------
    def fuse(self, lidar_pts, depth_pts):
        """Nearest-neighbour fusion: match lidar<->depth, keep unmatched from both."""
        fused = []
        used_depth = set()

        for lp in lidar_pts:
            best_j = -1
            best_d = 1e9

            for j, dp in enumerate(depth_pts):
                if j in used_depth:
                    continue
                d = np.linalg.norm(lp - dp)
                if d < best_d:
                    best_d = d
                    best_j = j

            if best_j >= 0 and best_d <= self.match_distance:
                fp = 0.5 * (lp + depth_pts[best_j])
                used_depth.add(best_j)
            else:
                fp = lp

            fused.append(fp)

        for j, dp in enumerate(depth_pts):
            if j not in used_depth:
                fused.append(dp)


        return fused

    def build_output(self, fused_pts, stamp):
        msg = PoseArray()
        msg.header.stamp = stamp if stamp != rospy.Time() else rospy.Time.now()
        msg.header.frame_id = self.target_frame

        for pt in fused_pts:
            pose = Pose()
            pose.position.x = float(pt[0])
            pose.position.y = float(pt[1])
            pose.position.z = 0.0
            pose.orientation.w = 1.0
            msg.poses.append(pose)

        return msg

    # ------------------------------------------------------------------
    # Synchronized callback (preferred path when both sensors publish)
    # ------------------------------------------------------------------
    def synced_callback(self, lidar_msg, depth_msg):
        lidar_pts = self.transform_poses_to_frame(lidar_msg, self.target_frame)
        depth_pts = self.transform_poses_to_frame(depth_msg, self.target_frame)

        fused = self.fuse(lidar_pts, depth_pts)

        stamp = lidar_msg.header.stamp
        out = self.build_output(fused, stamp)

        if self.debug:
            rospy.loginfo_throttle(
                1.0,
                "fusion synced: lidar=%d depth=%d fused=%d",
                len(lidar_pts),
                len(depth_pts),
                len(fused),
            )

        self.pub.publish(out)

    # ------------------------------------------------------------------
    # Fallback: publish when only one sensor is available
    # ------------------------------------------------------------------
    def fallback_publish(self, event):
        now = rospy.Time.now()
        lidar_age = (now - self.lidar_stamp).to_sec()
        depth_age = (now - self.depth_stamp).to_sec()

        # If both sensors published recently the synced callback handles it
        if lidar_age < self.fallback_timeout and depth_age < self.fallback_timeout:
            return

        stamp = now
        fused = []

        if self.lidar_msg is not None and lidar_age < self.fallback_timeout:
            fused = self.transform_poses_to_frame(self.lidar_msg, self.target_frame)
            stamp = self.lidar_msg.header.stamp
        elif self.depth_msg is not None and depth_age < self.fallback_timeout:
            fused = self.transform_poses_to_frame(self.depth_msg, self.target_frame)
            stamp = self.depth_msg.header.stamp

        if not fused:
            return

        out = self.build_output(fused, stamp)

        if self.debug:
            rospy.loginfo_throttle(
                2.0,
                "fusion fallback: lidar_age=%.2f depth_age=%.2f pts=%d",
                lidar_age,
                depth_age,
                len(fused),
            )

        self.pub.publish(out)


if __name__ == "__main__":
    try:
        node = ConeFusionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
