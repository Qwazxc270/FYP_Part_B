#! /usr/bin/env python3
"""
Path visualizer -- republishes the guidance trajectory as an RViz Marker.

Add a Marker display in RViz on topic /guidance_path_marker to see the
planned path as a green line strip alongside Ethan's cone markers.
"""

import rospy
from qcar_guidance.msg import TrajectoryMessage
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

pub = None


def traj_callback(msg):
    m = Marker()
    m.header.frame_id = "odom"
    m.header.stamp = rospy.Time.now()
    m.ns = "guidance_path"
    m.id = 0
    m.type = Marker.LINE_STRIP
    m.action = Marker.ADD
    m.scale.x = 0.05
    m.color.g = 1.0
    m.color.a = 1.0
    m.pose.orientation.w = 1.0
    for x, y in zip(msg.waypoint_x, msg.waypoint_y):
        pt = Point()
        pt.x, pt.y, pt.z = x, y, 0.05
        m.points.append(pt)
    pub.publish(m)


if __name__ == '__main__':
    rospy.init_node('path_visualizer_node')
    pub = rospy.Publisher('/guidance_path_marker', Marker, queue_size=1)
    rospy.Subscriber('/qcar/trajectory_topic', TrajectoryMessage, traj_callback)
    rospy.loginfo("path_visualizer_node started")
    rospy.spin()
