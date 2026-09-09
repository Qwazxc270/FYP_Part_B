#!/usr/bin/env python3

import math
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, Pose
from visualization_msgs.msg import Marker, MarkerArray


class ConeMapperNode:
    def __init__(self):
        rospy.init_node("cone_mapper_node")

        self.odom_topic = rospy.get_param("~odom_topic", "/odometry/filtered")
        self.detections_topic = rospy.get_param("~detections_topic", "/cone_detections_relative")
        self.map_topic = rospy.get_param("~map_topic", "/cone_map")
        self.marker_topic = rospy.get_param("~marker_topic", "/cone_map_markers")
        self.world_frame = rospy.get_param("~world_frame", "odom")

        self.association_distance = rospy.get_param("~association_distance", 0.60)
        self.min_range = rospy.get_param("~min_range", 0.1)
        self.max_range = rospy.get_param("~max_range", 8.0)

        self.sigma_range_base = rospy.get_param("~sigma_range_base", 0.08)
        self.sigma_range_scale = rospy.get_param("~sigma_range_scale", 0.03)
        self.sigma_bearing = rospy.get_param("~sigma_bearing_deg", 4.0)*np.pi/180.0

        self.pose_ready = False
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.P_pose = np.diag([1.0, 1.0, np.deg2rad(10.0)**2])

        self.landmarks = []

        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=1)
        rospy.Subscriber(self.detections_topic, PoseArray, self.detections_callback, queue_size=1)

        self.map_pub = rospy.Publisher(self.map_topic, PoseArray, queue_size=1)
        self.marker_pub = rospy.Publisher(self.marker_topic, MarkerArray, queue_size=1)

        rospy.loginfo("cone_mapper_node started")

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2.0*(q.w*q.z + q.x*q.y)
        cosy_cosp = 1.0 - 2.0*(q.y*q.y + q.z*q.z)
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

        for pose in msg.poses:
            z_body = np.array([[pose.position.x], [pose.position.y]])
            rng = float(np.linalg.norm(z_body))

            if rng < self.min_range or rng > self.max_range:
                continue

            mu_world, R_world = self.body_detection_to_world(z_body)
            self.associate_and_update(mu_world, R_world)

        stamp = msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time.now()
        self.publish_map(stamp)

    def body_detection_to_world(self, z_body):
        bx = z_body[0, 0]
        by = z_body[1, 0]

        rng = np.hypot(bx, by)
        bearing = np.arctan2(by, bx)

        sigma_r = self.sigma_range_base + self.sigma_range_scale*rng
        sigma_b = self.sigma_bearing
        R_rb = np.diag([sigma_r**2, sigma_b**2])

        J_rb_to_xy = np.array([
            [np.cos(bearing), -rng*np.sin(bearing)],
            [np.sin(bearing), rng*np.cos(bearing)],
        ])

        R_body = J_rb_to_xy@R_rb@J_rb_to_xy.T

        c = np.cos(self.yaw)
        s = np.sin(self.yaw)

        R_wb = np.array([[c, -s], [s, c]])
        t_wb = np.array([[self.x], [self.y]])

        mu_world = t_wb + R_wb@z_body

        J_pose = np.array([
            [1.0, 0.0, -s*bx - c*by],
            [0.0, 1.0, c*bx - s*by],
        ])


        R_world = R_wb@R_body@R_wb.T + J_pose@self.P_pose@J_pose.T
        return mu_world, R_world

    def associate_and_update(self, z_world, R_world):
        if not self.landmarks:
            self.add_landmark(z_world, R_world)
            return
        best_idx = -1
        best_dist = float("inf")

        for i, lm in enumerate(self.landmarks):
            dist = float(np.linalg.norm(z_world - lm["mu"]))
            if dist < best_dist:
                best_dist = dist
                best_idx = i

        if best_dist > self.association_distance:
            self.add_landmark(z_world, R_world)
            return

        lm = self.landmarks[best_idx]
        H = np.eye(2)
        S = H@lm["P"]@H.T + R_world
        K = lm["P"]@H.T@np.linalg.inv(S)
        innovation = z_world - lm["mu"]

        lm["mu"] = lm["mu"] + K@innovation
        I = np.eye(2)
        lm["P"] = (I - K@H)@lm["P"]@(I - K@H).T + K@R_world@K.T
        lm["hits"] += 1

    def add_landmark(self, mu_world, P_world):
        self.landmarks.append({
            "mu": np.array(mu_world, dtype=float),
            "P": np.array(P_world, dtype=float),
            "hits": 1,
        })

    def publish_map(self, stamp):
        pose_array = PoseArray()
        pose_array.header.stamp = stamp
        pose_array.header.frame_id = self.world_frame

        marker_array = MarkerArray()

        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        marker_array.markers.append(delete_all)

        for i, lm in enumerate(self.landmarks):
            px = lm["mu"][0, 0]
            py = lm["mu"][1, 0]
            P = lm["P"]

            pose = Pose()
            pose.position.x = px
            pose.position.y = py
            pose.position.z = 0.0
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)

            cone_marker = Marker()
            cone_marker.header.stamp = stamp
            cone_marker.header.frame_id = self.world_frame
            cone_marker.ns = "cones"
            cone_marker.id = i
            cone_marker.type = Marker.SPHERE
            cone_marker.action = Marker.ADD
            cone_marker.pose.position.x = px
            cone_marker.pose.position.y = py
            cone_marker.pose.position.z = 0.1
            cone_marker.pose.orientation.w = 1.0
            cone_marker.scale.x = 0.18
            cone_marker.scale.y = 0.18
            cone_marker.scale.z = 0.18
            cone_marker.color.a = 1.0
            cone_marker.color.r = 1.0
            cone_marker.color.g = 0.5
            cone_marker.color.b = 0.0
            marker_array.markers.append(cone_marker)

            cov_marker = Marker()
            cov_marker.header.stamp = stamp
            cov_marker.header.frame_id = self.world_frame
            cov_marker.ns = "cone_covariance"
            cov_marker.id = 1000 + i
            cov_marker.type = Marker.CYLINDER
            cov_marker.action = Marker.ADD
            cov_marker.pose.position.x = px
            cov_marker.pose.position.y = py 
            cov_marker.pose.position.z = 0.02
            cov_marker.pose.orientation.w = 1.0
            cov_marker.scale.x = max(0.05, 2.0*np.sqrt(max(P[0, 0], 1e-6)))
            cov_marker.scale.y = max(0.05, 2.0*np.sqrt(max(P[1, 1], 1e-6)))
            cov_marker.scale.z = 0.02
            cov_marker.color.a = 0.25
            cov_marker.color.r = 0.1
            cov_marker.color.g = 0.8
            cov_marker.color.b = 0.1
            marker_array.markers.append(cov_marker)

        self.map_pub.publish(pose_array)
        self.marker_pub.publish(marker_array)


if __name__ == "__main__":
    try:
        node = ConeMapperNode()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

