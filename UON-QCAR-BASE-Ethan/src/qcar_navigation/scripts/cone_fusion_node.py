#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import PoseArray, Pose

class ConeFusionNode:
    def __init__(self):
        rospy.init_node("cone_fusion_node")

        self.lidar_topic = rospy.get_param("~lidar_topic", "/cone_detections_lidar")
        self.depth_topic = rospy.get_param("~depth_topic", "/cone_detections_depth")
        self.output_topic = rospy.get_param("~output_topic", "/cone_detections_fused")
        self.match_distance = rospy.get_param("~match_distance", 0.3)

        self.lidar_msg = None
        self.depth_msg = None

        rospy.Subscriber(self.lidar_topic, PoseArray, self.lidar_callback, queue_size=1)
        rospy.Subscriber(self.depth_topic, PoseArray, self.depth_callback, queue_size=1)
        self.pub = rospy.Publisher(self.output_topic, PoseArray, queue_size=1)

        rospy.Timer(rospy.Duration(0.1), self.publish_fused)

        rospy.loginfo("cone_fusion_node started")

    def lidar_callback(self, msg):
        self.lidar_msg = msg

    def depth_callback(self, msg):
        self.depth_msg = msg

    def posearray_to_points(self, msg):
        pts = []
        if msg is None:
            return pts
        for p in msg.poses:
            pts.append(np.array([p.position.x, p.position.y], dtype=float))
        return pts

    def publish_fused(self, event):
        lidar_pts = self.posearray_to_points(self.lidar_msg)
        depth_pts = self.posearray_to_points(self.depth_msg)

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
                fp = 0.5*(lp + depth_pts[best_j])
                used_depth.add(best_j)
            else:
                fp = lp

            fused.append(fp)

        msg = PoseArray()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_footprint"

        for pt in fused:
            pose = Pose()
            pose.position.x = float(pt[0])
            pose.position.y = float(pt[1])
            pose.position.z = 0.0
            pose.orientation.w = 1.0
            msg.poses.append(pose)

        self.pub.publish(msg)

if __name__ == "__main__":
    try:
        node = ConeFusionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass