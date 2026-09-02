#!/usr/bin/env python3
import rospy 
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32, Float64
from geometry_msgs.msg import Vector3Stamped

class JoyTeleop:
    def __init__(self):
        rospy.init_node('joy_teleop')
        
        self.use_hardware = rospy.get_param('~use_hardware', False)
        
        if self.use_hardware:
            self.pub_cmd = rospy.Publisher('/qcar/cmd_vel', Vector3Stamped, queue_size=1)
            rospy.loginfo("Joy teleop ready - REAL HARDWARE mode. LS=forward/back, RS=steer")
        else:
            self.pub_rr = rospy.Publisher('/wheelrr_motor/command', Float32, queue_size=1)
            self.pub_rl = rospy.Publisher('/wheelrl_motor/command', Float32, queue_size=1)
            self.pub_fr = rospy.Publisher('/wheelfr_motor/command', Float32, queue_size=1)
            self.pub_fl = rospy.Publisher('/wheelfl_motor/command', Float32, queue_size=1)
            self.pub_steer_r = rospy.Publisher('/qcar/base_fr_controller/command', Float64, queue_size=1)
            self.pub_steer_l = rospy.Publisher('/qcar/base_fl_controller/command', Float64, queue_size=1)
            rospy.loginfo("Joy teleop ready - GAZEBO mode. Hold L1, LS=forward/back, RS=steer")

        self.AXIS_DRIVE = 1    # Left stick up/down
        self.AXIS_STEER = 3    # Right stick left/right
        self.DEADMAN_BTN = 4   # L1 button
        self.MAX_DUTY = 10.0
        self.MAX_PWM = 0.1
        self.MAX_STEER = 0.5
        self.FILTER_TC = 0.3
        self.filtered_throttle = 0.0
        self.last_time = rospy.Time.now()

        rospy.Subscriber('/joy', Joy, self.joy_cb)
        rospy.spin()

    def joy_cb(self, msg):
        drive_input = msg.axes[self.AXIS_DRIVE] if len(msg.axes) > self.AXIS_DRIVE else 0.0
        steer_input = msg.axes[self.AXIS_STEER] if len(msg.axes) > self.AXIS_STEER else 0.0

        if self.use_hardware:
            target = drive_input * self.MAX_PWM
            steer = steer_input * self.MAX_STEER

            now = rospy.Time.now()
            dt = (now - self.last_time).to_sec()
            self.last_time = now
            alpha = dt / (self.FILTER_TC + dt) if dt > 0 else 1.0
            self.filtered_throttle += alpha * (target - self.filtered_throttle)

            cmd = Vector3Stamped()
            cmd.header.stamp = now
            cmd.header.frame_id = "base_link"
            cmd.vector.x = self.filtered_throttle
            cmd.vector.y = steer
            cmd.vector.z = 0.0
            self.pub_cmd.publish(cmd)

        else:
            if not msg.buttons[self.DEADMAN_BTN]:
                for pub in [self.pub_rr, self.pub_rl, self.pub_fr, self.pub_fl]:
                    pub.publish(Float32(0.0))
                self.pub_steer_r.publish(Float64(0.0))
                self.pub_steer_l.publish(Float64(0.0))
                return

            drive = drive_input * self.MAX_DUTY
            turn = steer_input * 0.3 if abs(drive) > 0.1 else 0.0
            steer = steer_input * self.MAX_STEER

            left = drive - turn
            right = drive + turn

            self.pub_rr.publish(Float32(right))
            self.pub_fr.publish(Float32(right))
            self.pub_rl.publish(Float32(-left))
            self.pub_fl.publish(Float32(-left))
            self.pub_steer_r.publish(Float64(steer))
            self.pub_steer_l.publish(Float64(steer))

if __name__ == '__main__':
    try:
        JoyTeleop()
    except rospy.ROSInterruptException:
        pass