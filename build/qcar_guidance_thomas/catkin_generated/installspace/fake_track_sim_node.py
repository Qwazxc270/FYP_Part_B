#! /usr/bin/env python3
"""
fake_track_sim_node -- closed-loop stand-in for Gazebo + navigation + control.

For the isolated launch test (launch/test_guidance_isolated.launch). Loads a
real qcar_gazebo world file, drives a simulated car along whatever
/qcar/trajectory_topic says (pure pursuit, standing in for the controller),
and publishes the same topics as the real navigation stack:

  /odometry/local                  Odometry   smooth, slowly drifting
  /odometry/filtered               Odometry   drift-free, jumps like EKF-SLAM
  /cone_detections_fused_coloured  ConeDetectionArray  camera (FOV-limited)
  /cone_detections_fused           PoseArray  lidar (all round, no colour)
  /cone_map, /cone_map_markers     SLAM-style map of cones seen so far
  /fake_sim/cones, /fake_sim/car   RViz markers (ground truth)

Not Gazebo physics and not Luke's MPC: this checks the guidance node's ROS
plumbing and logic end to end in seconds, nothing more.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np                                             # noqa: E402
import rospy                                                   # noqa: E402
from geometry_msgs.msg import Pose, PoseArray, Point           # noqa: E402
from nav_msgs.msg import Odometry                              # noqa: E402
from std_msgs.msg import Header                                # noqa: E402
from visualization_msgs.msg import Marker, MarkerArray         # noqa: E402
from qcar_guidance.msg import TrajectoryMessage                # noqa: E402
from qcar_navigation.msg import ConeDetection, ConeDetectionArray  # noqa: E402

import track_sim as ts                                         # noqa: E402

RGB = {ts.BLUE: (0.0, 0.3, 1.0), ts.YELLOW: (1.0, 0.9, 0.0),
       ts.ORANGE: (1.0, 0.4, 0.0), ts.UNKNOWN: (0.6, 0.6, 0.6)}


def odom_msg(pose, v, stamp):
    m = Odometry()
    m.header = Header(stamp=stamp, frame_id='odom')
    m.child_frame_id = 'base_footprint'
    m.pose.pose.position.x, m.pose.pose.position.y = pose[0], pose[1]
    m.pose.pose.orientation.z = math.sin(pose[2] / 2)
    m.pose.pose.orientation.w = math.cos(pose[2] / 2)
    m.twist.twist.linear.x = v
    return m


class FakeSim:
    def __init__(self):
        track = rospy.get_param('~track', 'Track1')
        world = rospy.get_param('~world', '')
        if not world:
            import rospkg
            world = os.path.join(rospkg.RosPack().get_path('qcar_gazebo'),
                                 'worlds', track + '.world')
        self.cones = ts.load_world_cones(world)
        spawn = ts.SPAWN.get(track, (0.0, 0.0, 0.0))
        self.car = ts.SimCar(spawn)
        self.odo = ts.OdometryModel(
            spawn, drift_yaw_rate=math.radians(rospy.get_param('~drift_deg_per_s', 0.3)),
            jump=rospy.get_param('~slam_jump', 0.10))
        self.sens = ts.Sensors(self.cones,
                               cam_fov=rospy.get_param('~camera_fov', 1.6),
                               cam_range=rospy.get_param('~camera_range', 3.0),
                               lidar_range=rospy.get_param('~lidar_range', 4.0),
                               phantom_period=rospy.get_param('~phantom_period', 15.0))
        self.local = spawn
        self.hits = set()
        self.hit_count = 0
        self.next_det = 0.0
        self.next_map = 0.0

        self.pub_local = rospy.Publisher('/odometry/local', Odometry, queue_size=10)
        self.pub_slam = rospy.Publisher('/odometry/filtered', Odometry, queue_size=10)
        self.pub_col = rospy.Publisher('/cone_detections_fused_coloured',
                                       ConeDetectionArray, queue_size=1)
        self.pub_plain = rospy.Publisher('/cone_detections_fused', PoseArray, queue_size=1)
        self.pub_map = rospy.Publisher('/cone_map', PoseArray, queue_size=1)
        self.pub_mark = rospy.Publisher('/cone_map_markers', MarkerArray, queue_size=1)
        self.pub_truth = rospy.Publisher('/fake_sim/cones', MarkerArray, queue_size=1, latch=True)
        self.pub_car = rospy.Publisher('/fake_sim/car', Marker, queue_size=1)
        rospy.Subscriber('/qcar/trajectory_topic', TrajectoryMessage, self._traj_cb,
                         queue_size=1)
        self._publish_truth()
        rospy.loginfo("fake_track_sim: %s, %d cones, spawn %s" % (track, len(self.cones), spawn))

    def _traj_cb(self, msg):
        """Trajectory arrives in the LOCAL frame (what the controller uses);
        convert through the exact local->true transform of this instant."""
        if len(msg.waypoint_x) < 2:
            return
        lx, ly, lyaw = self.local
        tx, ty, tyaw = self.car.pose
        d = ts.wrap(tyaw - lyaw)
        c, s = math.cos(d), math.sin(d)
        pts = [(tx + c * (x - lx) - s * (y - ly), ty + s * (x - lx) + c * (y - ly))
               for x, y in zip(msg.waypoint_x, msg.waypoint_y)]
        self.car.set_command(np.array(pts).T, msg.velocity)

    def _publish_truth(self):
        ma = MarkerArray()
        for i, (x, y, col) in enumerate(self.cones):
            m = Marker()
            m.header.frame_id = 'odom'
            m.ns, m.id, m.type = 'truth', i, Marker.CYLINDER
            m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, 0.1
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = 0.15
            m.scale.z = 0.2
            m.color.r, m.color.g, m.color.b = RGB[col]
            m.color.a = 0.5
            ma.markers.append(m)
        self.pub_truth.publish(ma)

    def _detections(self, cam, lidar, stamp):
        hdr = Header(stamp=stamp, frame_id='base_footprint')
        ca = ConeDetectionArray(header=hdr)
        for x, y, col in cam:
            d = ConeDetection()
            d.header = hdr
            d.colour = col
            d.position.x, d.position.y = x, y
            d.range = math.hypot(x, y)
            ca.detections.append(d)
        pa = PoseArray(header=hdr)
        for x, y, _ in lidar:
            p = Pose()
            p.position.x, p.position.y = x, y
            p.orientation.w = 1.0
            pa.poses.append(p)
        self.pub_col.publish(ca)
        self.pub_plain.publish(pa)

    def _map(self, stamp):
        cones = self.sens.slam_map()
        pa = PoseArray(header=Header(stamp=stamp, frame_id='odom'))
        ma = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        ma.markers.append(clear)
        for i, (x, y, col) in enumerate(cones):
            p = Pose()
            p.position.x, p.position.y = x, y
            p.orientation.w = 1.0
            pa.poses.append(p)
            m = Marker()
            m.header = pa.header
            m.ns, m.id, m.type, m.action = 'cones', i, Marker.SPHERE, Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, 0.1
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.18
            m.color.r, m.color.g, m.color.b = RGB[col]
            m.color.a = 1.0
            ma.markers.append(m)
        self.pub_map.publish(pa)
        self.pub_mark.publish(ma)

    def tick(self, dt, t):
        """Advance the simulation by dt and publish everything due at time t."""
        stamp = rospy.Time.now()
        self.car.step(dt)
        self.local, slam = self.odo.update(self.car.pose, dt, t)
        self.pub_local.publish(odom_msg(self.local, self.car.v, stamp))
        self.pub_slam.publish(odom_msg(slam, self.car.v, stamp))
        if t >= self.next_det:
            self.next_det = t + 0.1
            cam, lidar, _ = self.sens.detect(self.car.pose, t)
            self._detections(cam, lidar, stamp)
        if t >= self.next_map:
            self.next_map = t + 0.5
            self._map(stamp)
        for i, (cx, cy, _) in enumerate(self.cones):
            d = math.hypot(cx - self.car.x, cy - self.car.y)
            if d < 0.17 and i not in self.hits:
                self.hits.add(i)
                self.hit_count += 1
                rospy.logwarn("fake_track_sim: CONE HIT at (%.2f, %.2f) -- total %d"
                              % (cx, cy, self.hit_count))
            elif i in self.hits and d > 0.5:
                self.hits.discard(i)
        car = Marker()
        car.header = Header(stamp=stamp, frame_id='odom')
        car.ns, car.id, car.type = 'car', 0, Marker.ARROW
        car.pose.position.x, car.pose.position.y = self.car.x, self.car.y
        car.pose.orientation.z = math.sin(self.car.yaw / 2)
        car.pose.orientation.w = math.cos(self.car.yaw / 2)
        car.scale.x, car.scale.y, car.scale.z = 0.4, 0.15, 0.1
        car.color.g, car.color.a = 1.0, 1.0
        self.pub_car.publish(car)

    def spin(self):
        dt = 0.02
        rate = rospy.Rate(1.0 / dt)
        t = 0.0
        while not rospy.is_shutdown():
            self.tick(dt, t)
            t += dt
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('fake_track_sim')
    FakeSim().spin()
