#! /usr/bin/env python3
"""
Fake cone publisher -- isolated guidance testing (no Gazebo, no navigation).

Publishes cone detections along a curved test track, revealing cones
gradually (only those within SENSOR_RANGE of the fake car) to mimic real
incremental perception, plus matching odometry as the fake car crawls
along the centreline.

Track layout: 4 m straight, then a 90-degree left curve (R = 3 m),
cones offset +/- 1 m each side of the centreline.
Blue = left boundary, yellow = right boundary (FSAE convention).
"""

import rospy
import math
from qcar_navigation.msg import ConeDetectionArray, ConeDetection
from nav_msgs.msg import Odometry

SENSOR_RANGE = 4.0   # cones within this range are "detected" (m)
CAR_SPEED = 0.3      # fake car speed along the centreline (m/s)
STEP = 0.8           # centreline sample spacing (m)


def make_track():
    centre = []
    for d in [i * STEP for i in range(6)]:          # straight section
        centre.append((d, 0.0, 0.0))                # (x, y, heading)
    R = 3.0
    for a in [i * 0.15 for i in range(1, 12)]:      # curve section
        cx, cy = 4.0, R
        x = cx + R * math.sin(a)
        y = cy - R * math.cos(a)
        centre.append((x, y, a))

    blue, yellow = [], []
    for (x, y, h) in centre:
        lx, ly = -math.sin(h), math.cos(h)          # left normal
        blue.append((x + lx, y + ly))
        yellow.append((x - lx, y - ly))
    return centre, blue, yellow


def main():
    rospy.init_node('fake_cone_publisher')
    cone_pub = rospy.Publisher('/cone_detections_fused_coloured',
                               ConeDetectionArray, queue_size=1)
    odom_pub = rospy.Publisher('/odometry/filtered', Odometry, queue_size=1)

    centre, blue_cones, yellow_cones = make_track()
    progress = 0.0

    rate = rospy.Rate(2)
    t_last = rospy.get_time()
    while not rospy.is_shutdown():
        t = rospy.get_time()
        progress += CAR_SPEED * (t - t_last)
        t_last = t

        idx = min(int(progress / STEP), len(centre) - 1)
        car_x, car_y, car_h = centre[idx]

        odom = Odometry()
        odom.header.frame_id = "odom"
        odom.pose.pose.position.x = car_x
        odom.pose.pose.position.y = car_y
        odom.pose.pose.orientation.z = math.sin(car_h / 2)
        odom.pose.pose.orientation.w = math.cos(car_h / 2)
        odom_pub.publish(odom)

        msg = ConeDetectionArray()
        msg.header.frame_id = "odom"   # fixed frame: detections_in_car_frame=false
        for cones, colour in ((blue_cones, ConeDetection.BLUE),
                              (yellow_cones, ConeDetection.YELLOW)):
            for (x, y) in cones:
                if math.dist((car_x, car_y), (x, y)) <= SENSOR_RANGE:
                    det = ConeDetection()
                    det.position.x = x
                    det.position.y = y
                    det.colour = colour
                    msg.detections.append(det)
        cone_pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    main()
