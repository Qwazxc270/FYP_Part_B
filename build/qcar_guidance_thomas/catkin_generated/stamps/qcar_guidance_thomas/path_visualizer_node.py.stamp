#! /usr/bin/env python3
"""
path_visualizer_node -- the current trajectory as an RViz line (green).

RViz: Add -> By topic -> /guidance_path_marker (Marker). The optimised race
line is published separately by the guidance node on /guidance_race_line
(magenta) once lap 1 is complete.
"""
import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from qcar_guidance.msg import TrajectoryMessage

pub = None
FRAME = 'odom'


def traj_callback(msg):
    m = Marker()
    m.header.frame_id = FRAME
    m.header.stamp = rospy.Time.now()
    m.ns, m.id = 'guidance_path', 0
    m.type, m.action = Marker.LINE_STRIP, Marker.ADD
    m.scale.x = 0.05
    m.color.g, m.color.a = 1.0, 1.0
    m.pose.orientation.w = 1.0
    for x, y in zip(msg.waypoint_x, msg.waypoint_y):
        m.points.append(Point(x=x, y=y, z=0.05))
    pub.publish(m)


if __name__ == '__main__':
    rospy.init_node('path_visualizer_node')
    FRAME = rospy.get_param('~frame_id', 'odom')
    pub = rospy.Publisher('/guidance_path_marker', Marker, queue_size=1)
    rospy.Subscriber('/qcar/trajectory_topic', TrajectoryMessage, traj_callback)
    rospy.spin()
