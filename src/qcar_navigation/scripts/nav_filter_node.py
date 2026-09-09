#!/usr/bin/env python3

import rospy
import numpy as np
import math
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float64, Float32
import tf.transformations as tft

def WrapToPi(angle):


    wrapped = np.mod(angle + np.pi,2*np.pi)
    if wrapped == 0 and (angle + np.pi) > 0:
        wrapped = 2*np.pi

    wrapped = wrapped - np.pi

    if wrapped == -np.pi and angle > 0:
        wrapped = np.pi
    return wrapped



class NavEKF:
    def __init__(self, x0):
        # state = [X, Y, psi, v]^T

        self.x = np.array(x0, dtype=float).reshape(4, 1)
        sigma_x = 0.2
        sigma_y = 0.2
        sigma_psi = np.deg2rad(5)
        sigma_v = 0.2

        self.P = np.diag([sigma_x**2, sigma_y**2, sigma_psi**2,sigma_v**2])

        self.Q = np.diag([0.03**2, 0.03**2, np.deg2rad(1)**2, 0.1**2])

        self.R = np.diag([np.deg2rad(0.8)**2, 0.03**2])

        self.yaw_rate_meas = 0.0
        self.v_meas = 0.0

        


    def ekfUpdate(self, delta, L):

        x = self.x
        P = self.P
        R = self.R

        v = x[3, 0]

        yhat = np.array([[self.yaw_rate_meas],
                        [self.v_meas]])

        y = np.array([[(v/L)*np.tan(delta)],
                    [v] ])

        C = np.zeros((2,len(x)))
        C[:, 0:4] = np.array([[0,0,0,np.tan(delta)/L],
                            [0,0,0,1]])

        e = yhat - y

        S = C@P@C.T + R

        K = (P@C.T)@np.linalg.inv(S)

        x = x + K@e
        x[2, 0] = WrapToPi(x[2, 0])

        I = np.eye(P.shape[0])

        self.x = x
        self.P = (I - K@C)@P@(I - K@C).T + K@R@K.T


    def ekfUpdateYaw(self, yaw_meas, R_yaw):
        x = self.x
        P = self.P

        H = np.array([[0.0, 0.0, 1.0, 0.0]])

        y_pred = np.array([[x[2, 0]]])
        innovation = np.array([[WrapToPi(yaw_meas - y_pred[0, 0])]])

        S = H @ P @ H.T + np.array([[R_yaw]])
        K = P @ H.T @ np.linalg.inv(S)

        x = x + K @ innovation
        x[2, 0] = WrapToPi(x[2, 0])

        I = np.eye(P.shape[0])

        self.x = x
        self.P = (I - K @ H) @ P @ (I - K @ H).T + K @ np.array([[R_yaw]]) @ K.T


    def ekfUpdateXY(self, x_meas, y_meas, R_xy):
        x = self.x
        P = self.P

        H = np.array([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0]
        ])

        z = np.array([
            [x_meas],
            [y_meas]
        ])

        z_pred = np.array([
            [x[0, 0]],
            [x[1, 0]]
        ])

        innovation = z - z_pred

        R = np.diag([R_xy, R_xy])

        S = H @ P @ H.T + R
        K = P @ H.T @ np.linalg.inv(S)

        x = x + K @ innovation

        I = np.eye(P.shape[0])

        self.x = x
        self.P = (I - K @ H) @ P @ (I - K @ H).T + K @ R @ K.T

    

    def ekfPredict(self, u, dt, L):

        x = self.x
        P = self.P


        #n = len(x)

        X = x[0, 0]
        Y = x[1, 0]
        psi = x[2, 0]
        v = x[3, 0]

        delta = u[0, 0]
        a = u[1, 0]

        x[0, 0] = X + v*np.cos(psi)*dt
        x[1, 0] = Y + v*np.sin(psi)*dt
        x[2, 0] = WrapToPi(psi + (v/L)*np.tan(delta)*dt)
        x[3, 0] = v + a*dt


        F_Jac = np.array([
            [1, 0, -v*np.sin(psi)*dt, np.cos(psi)*dt],
            [0, 1, v*np.cos(psi)*dt, np.sin(psi)*dt],
            [0, 0, 1, (1/L)*np.tan(delta)*dt],
            [0, 0, 0, 1]
        ])

        self.x = x
        self.P = F_Jac@P@F_Jac.T + self.Q



class NavFilterNode:
    def __init__(self):
        rospy.init_node("nav_filter_node")

        self.latest_imu = None
        self.latest_odom = None

        self.w_rl = 0.0
        self.w_rr = 0.0

        self.delta_fl = 0.0
        self.delta_fr = 0.0

        self.pre_v_meas = None
        self.last_time = None
        self.initialized = False

        self.wheel_radius = 0.0325
        self.L = 0.26
        self.R_yaw_abs = np.deg2rad(2.0)**2
        self.R_xy_abs = 0.05**2

        self.ekf = None

        rospy.Subscriber("/imu", Imu, self.imu_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/wheelrl_motor/velocity", Float32, self.wheel_rl_callback)
        rospy.Subscriber("/wheelrr_motor/velocity", Float32, self.wheel_rr_callback)
        rospy.Subscriber("/qcar/base_fl_controller/command", Float64, self.steer_fl_callback)
        rospy.Subscriber("/qcar/base_fr_controller/command", Float64, self.steer_fr_callback)

        self.odom_pub = rospy.Publisher("/odometry/filtered", Odometry, queue_size=10)

    def imu_callback(self, msg):
        self.latest_imu = msg

    def odom_callback(self, msg):
        self.latest_odom = msg
        
    def wheel_rl_callback(self, msg):
        self.w_rl = msg.data

    def wheel_rr_callback(self, msg):
        self.w_rr = msg.data

    def steer_fl_callback(self, msg):
        self.delta_fl = msg.data

    def steer_fr_callback(self, msg):
        self.delta_fr = msg.data


    def odom_to_yaw(self, odom_msg):
        q = odom_msg.pose.pose.orientation
        quat = [q.x, q.y, q.z, q.w]
        _, _, yaw = tft.euler_from_quaternion(quat)
        return yaw

    def get_steering_angle(self):
        delta = 0.5 * (self.delta_fl + self.delta_fr)
        return delta

    def compute_speed_measurement(self):
        v_meas = self.wheel_radius * 0.5 * (-self.w_rl + self.w_rr)
        return v_meas

    def initialize_filter(self):
        if self.latest_odom is None:
            return False

        print("odom x =", self.latest_odom.pose.pose.position.x)
        print("odom y =", self.latest_odom.pose.pose.position.y)
        print("odom yaw =", self.odom_to_yaw(self.latest_odom))


        x0 = self.latest_odom.pose.pose.position.x
        y0 = self.latest_odom.pose.pose.position.y
        psi0 = self.odom_to_yaw(self.latest_odom)
        v0 = 0.0
        

        self.ekf = NavEKF([x0, y0, psi0, v0])
        self.initialized = True


        rospy.loginfo("EKF initialized: x=%.3f y=%.3f psi=%.3f v=%.3f", x0, y0, psi0, v0)

        return True


    def publish_filtered_odom(self, stamp, delta):
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_footprint"

        x = self.ekf.x[0, 0]
        y = self.ekf.x[1, 0]
        yaw = self.ekf.x[2, 0]
        v = self.ekf.x[3, 0]

        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0

        q = tft.quaternion_from_euler(0.0, 0.0, yaw)
        msg.pose.pose.orientation = Quaternion(*q)

        msg.twist.twist.linear.x = v
        msg.twist.twist.linear.y = 0.0
        msg.twist.twist.linear.z = 0.0
        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        msg.twist.twist.angular.z = (v/self.L)*np.tan(delta)

        self.odom_pub.publish(msg)



    def run(self):
        rate = rospy.Rate(50)
        

        while not rospy.is_shutdown():
            now = rospy.Time.now()

            if self.last_time is None:
                self.last_time = now
                rate.sleep()
                continue

            dt = (now - self.last_time).to_sec()
            self.last_time = now

            if dt <= 0.0:
                rate.sleep()
                continue

            if not self.initialized:
                self.initialize_filter()
                rate.sleep()
                continue

            delta = 0.63*self.get_steering_angle()
            delta = np.clip(delta, -0.23, 0.23)
            v_meas = self.compute_speed_measurement()

            if self.pre_v_meas is None:
                a = 0.0
            else:
                a = (v_meas - self.pre_v_meas)/dt

            self.pre_v_meas = v_meas

            u = np.array([
                [delta],
                [a]
            ])

            if self.latest_imu is None:
                rate.sleep()
                continue

            self.ekf.yaw_rate_meas = self.latest_imu.angular_velocity.z

            self.ekf.v_meas = v_meas

            self.ekf.ekfPredict(u, dt, self.L)
            self.ekf.ekfUpdate(delta, self.L)

            yaw_meas = self.odom_to_yaw(self.latest_odom)
            self.ekf.ekfUpdateYaw(yaw_meas, self.R_yaw_abs)
           

            rospy.loginfo_throttle(
                0.5,
                "odom x=%.3f y=%.3f yaw=%.3f vx=%.3f | ekf x=%.3f y=%.3f psi=%.3f v=%.3f | v_meas=%.3f delta=%.3f w_rl=%.3f w_rr=%.3f"
                % (
                    self.latest_odom.pose.pose.position.x,
                    self.latest_odom.pose.pose.position.y,
                    self.odom_to_yaw(self.latest_odom),
                    self.latest_odom.twist.twist.linear.x,
                    self.ekf.x[0, 0],
                    self.ekf.x[1, 0],
                    self.ekf.x[2, 0],
                    self.ekf.x[3, 0],
                    v_meas,
                    delta,
                    self.w_rl,
                    self.w_rr
                )
            )
            self.publish_filtered_odom(now, delta)
            rate.sleep()

if __name__ == "__main__":
    try:
        node = NavFilterNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
    