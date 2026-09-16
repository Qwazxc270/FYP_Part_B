#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, Pose
from visualization_msgs.msg import Marker, MarkerArray
from qcar_navigation.msg import ConeDetection, ConeDetectionArray


class ConeMapperNode:
    def __init__(self):
        rospy.init_node("cone_mapper_node")

        self.odom_topic = rospy.get_param("~odom_topic", "/odometry/filtered")
        self.detections_topic = rospy.get_param("~detections_topic", "/cone_detections_fused")
        self.map_topic = rospy.get_param("~map_topic", "/cone_map")
        self.marker_topic = rospy.get_param("~marker_topic", "/cone_map_markers")
        self.world_frame = rospy.get_param("~world_frame", "odom")

        self.association_distance = rospy.get_param("~association_distance", 0.50)
        self.candidate_association_distance = rospy.get_param("~candidate_association_distance", 0.45)
        self.min_landmark_spacing = rospy.get_param("~min_landmark_spacing", 0.75)
        self.confirmation_hits = rospy.get_param("~confirmation_hits", 3)
        self.candidate_max_age = rospy.get_param("~candidate_max_age", 12)
        self.min_motion_for_rehit = rospy.get_param("~min_motion_for_rehit", 0.20)
        self.min_range = rospy.get_param("~min_range", 0.1)
        self.max_range = rospy.get_param("~max_range", 4.5)
        self.sigma_range_base = rospy.get_param("~sigma_range_base", 0.10)
        self.sigma_range_scale = rospy.get_param("~sigma_range_scale", 0.04)
        self.sigma_bearing = rospy.get_param("~sigma_bearing_deg", 5.0) * np.pi / 180.0

        self.pose_ready = False
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.P_pose = np.diag([1.0, 1.0, np.deg2rad(10.0) ** 2])

        self.landmarks = []
        self.candidates = []
        self.published_ids = set()

        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=1)
        rospy.Subscriber(self.detections_topic, PoseArray, self.detections_callback, queue_size=1)

        self.map_pub = rospy.Publisher(self.map_topic, PoseArray, queue_size=1)
        self.marker_pub = rospy.Publisher(self.marker_topic, MarkerArray, queue_size=1)

        rospy.loginfo("cone_mapper_node started")

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = np.arctan2(siny_cosp, cosy_cosp)
        cov = np.array(msg.pose.covariance, dtype=float).reshape(6, 6)
        self.P_pose = np.array([
            [cov[0, 0], cov[0, 1], cov[0, 5]],
            [cov[1, 0], cov[1, 1], cov[1, 5]],
            [cov[5, 0], cov[5, 1], cov[5, 5]],
        ])
        self.pose_ready = True

    def detections_callback(self, msg):
        if not self.pose_ready:
            return
        current_pose = np.array([self.x, self.y], dtype=float)
        for pose in msg.poses:
            z_body = np.array([[pose.position.x], [pose.position.y]])
            rng = float(np.linalg.norm(z_body))
            if rng < self.min_range or rng > self.max_range:
                continue
            mu_world, R_world = self.body_detection_to_world(z_body)
            self.process_detection(mu_world, R_world, current_pose)
        self.age_candidates()
        stamp = msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time.now()
        self.publish_map(stamp)

    def body_detection_to_world(self, z_body):
        bx = z_body[0, 0]
        by = z_body[1, 0]
        rng = np.hypot(bx, by)
        bearing = np.arctan2(by, bx)
        sigma_r = self.sigma_range_base + self.sigma_range_scale * rng
        R_rb = np.diag([sigma_r ** 2, self.sigma_bearing ** 2])
        J_rb = np.array([
            [np.cos(bearing), -rng * np.sin(bearing)],
            [np.sin(bearing),  rng * np.cos(bearing)],
        ])
        R_body = J_rb @ R_rb @ J_rb.T
        c = np.cos(self.yaw)
        s = np.sin(self.yaw)
        R_wb = np.array([[c, -s], [s, c]])
        mu_world = np.array([[self.x], [self.y]]) + R_wb @ z_body
        J_pose = np.array([
            [1.0, 0.0, -s * bx - c * by],
            [0.0, 1.0,  c * bx - s * by],
        ])
        R_world = R_wb @ R_body @ R_wb.T + J_pose @ self.P_pose @ J_pose.T
        return mu_world, R_world

    def process_detection(self, z_world, R_world, current_pose):
        best_idx, best_dist = self.find_nearest(z_world, self.landmarks)
        if best_idx >= 0 and best_dist <= self.association_distance:
            lm = self.landmarks[best_idx]
            if self.pose_moved_enough(lm, current_pose):
                self.update_track(lm, z_world, R_world, current_pose)
            return
        if self.too_close_to_confirmed(z_world):
            return
        cand_idx, cand_dist = self.find_nearest(z_world, self.candidates)
        if cand_idx >= 0 and cand_dist <= self.candidate_association_distance:
            cand = self.candidates[cand_idx]
            if self.pose_moved_enough(cand, current_pose):
                self.update_track(cand, z_world, R_world, current_pose)
                cand["age"] = 0
                if cand["hits"] >= self.confirmation_hits and not self.too_close_to_confirmed(cand["mu"]):
                    self.promote_candidate(cand_idx)
            return
        self.add_candidate(z_world, R_world, current_pose)

    def find_nearest(self, z_world, tracks):
        if not tracks:
            return -1, float("inf")
        best_idx = -1
        best_dist = float("inf")
        for i, trk in enumerate(tracks):
            d = float(np.linalg.norm(z_world - trk["mu"]))
            if d < best_dist:
                best_dist = d
                best_idx = i
        return best_idx, best_dist

    def pose_moved_enough(self, track, current_pose):
        last = track.get("last_update_pose")
        if last is None:
            return True
        return np.linalg.norm(current_pose - last) >= self.min_motion_for_rehit

    def update_track(self, track, z_world, R_world, current_pose):
        H = np.eye(2)
        S = H @ track["P"] @ H.T + R_world
        K = track["P"] @ H.T @ np.linalg.inv(S)
        track["mu"] = track["mu"] + K @ (z_world - track["mu"])
        I = np.eye(2)
        track["P"] = (I - K @ H) @ track["P"] @ (I - K @ H).T + K @ R_world @ K.T
        track["hits"] += 1
        track["last_update_pose"] = current_pose.copy()

    def too_close_to_confirmed(self, z_world):
        return any(float(np.linalg.norm(z_world - lm["mu"])) < self.min_landmark_spacing
                   for lm in self.landmarks)

    def add_candidate(self, mu_world, P_world, current_pose):
        self.candidates.append({
            "mu": np.array(mu_world, dtype=float),
            "P": np.array(P_world, dtype=float),
            "hits": 1, "age": 0,
            "last_update_pose": current_pose.copy(),
        })

    def promote_candidate(self, idx):
        cand = self.candidates.pop(idx)
        self.landmarks.append({
            "mu": cand["mu"], "P": cand["P"],
            "hits": cand["hits"],
            "last_update_pose": cand["last_update_pose"],
        })

    def age_candidates(self):
        survivors = []
        for c in self.candidates:
            c["age"] += 1
            if c["age"] <= self.candidate_max_age:
                survivors.append(c)
        self.candidates = survivors

    def publish_map(self, stamp):
        pose_array = PoseArray()
        pose_array.header.stamp = stamp
        pose_array.header.frame_id = self.world_frame

        marker_array = MarkerArray()
        current_ids = set()

        for i, lm in enumerate(self.landmarks):
            px, py = lm["mu"][0, 0], lm["mu"][1, 0]
            P = lm["P"]

            pose = Pose()
            pose.position.x = px
            pose.position.y = py
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)

            m = Marker()
            m.header.stamp = stamp
            m.header.frame_id = self.world_frame
            m.ns = "cones"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = px
            m.pose.position.y = py
            m.pose.position.z = 0.3
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.18
            m.color.a = 1.0
            m.color.r = 1.0
            m.color.g = 0.5
            m.color.b = 0.0
            marker_array.markers.append(m)
            current_ids.add(i)

            cm = Marker()
            cm.header.stamp = stamp
            cm.header.frame_id = self.world_frame
            cm.ns = "cone_covariance"
            cm.id = 1000 + i
            cm.type = Marker.CYLINDER
            cm.action = Marker.ADD
            cm.pose.position.x = px
            cm.pose.position.y = py
            cm.pose.position.z = 0.02
            cm.pose.orientation.w = 1.0
            cm.scale.x = max(0.05, min(0.5, 2.0 * np.sqrt(max(P[0, 0], 1e-6))))
            cm.scale.y = max(0.05, min(0.5, 2.0 * np.sqrt(max(P[1, 1], 1e-6))))
            cm.scale.z = 0.02
            cm.color.a = 0.25
            cm.color.r = 0.1
            cm.color.g = 0.8
            cm.color.b = 0.1
            marker_array.markers.append(cm)

        for old_id in self.published_ids - current_ids:
            for ns, mid in [("cones", old_id), ("cone_covariance", 1000 + old_id)]:
                d = Marker()
                d.ns = ns
                d.id = mid
                d.action = Marker.DELETE
                marker_array.markers.append(d)
        self.published_ids = current_ids

        for i, cand in enumerate(self.candidates):
            cm = Marker()
            cm.header.stamp = stamp
            cm.header.frame_id = self.world_frame
            cm.ns = "cone_candidates"
            cm.id = 2000 + i
            cm.type = Marker.SPHERE
            cm.action = Marker.ADD
            cm.pose.position.x = cand["mu"][0, 0]
            cm.pose.position.y = cand["mu"][1, 0]
            cm.pose.position.z = 0.05
            cm.pose.orientation.w = 1.0
            cm.scale.x = cm.scale.y = cm.scale.z = 0.10
            cm.color.a = 0.4
            cm.color.r = 1.0
            marker_array.markers.append(cm)

        self.map_pub.publish(pose_array)
        self.marker_pub.publish(marker_array)


if __name__ == "__main__":
    try:
        node = ConeMapperNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass