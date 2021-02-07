#!/usr/bin/env python

# some of this libraries might not be necessary
from __future__ import print_function
import rospy
import math
import numpy as np
import roslib
import sys
import rospy
# import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
# from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion
# from geometry_msgs.msg import Transform
import time
import copy


# def commander():
#     rospy.init_node('wheel_command', anonymous=False)
#     rospy.Subscriber("sensor_msgs/Imu", Imu, callback)
#
#     rate = rospy.Rate(20)  # 10hz
#     msg2pub = Twist()
#     msg2pub.linear.x = 10
#     while not rospy.is_shutdown():
#         msg2pub.angular.z = 0
#         msg2pub.linear.x *= -1
#         pub.publish(msg2pub)
#         rate.sleep()


class robot_control:

    def __init__(self):
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.msg2pub = Twist()
        self.msg2pub.linear.x = 0

        self.wheel_radius = 0.1
        self.rotation = 0.0

        self.tita_target = 0.0
        self.delta_tita_buffer = [0.0, 0.0, 0.0]
        # self.old_delta_tita = 0.0
        self.omega = 0.0
        self.omega_buffer = [0.0, 0.0]
        # self.old_omega = 0.0

    def callback(self, data):
        orientation_quat = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
        orientation_euler = euler_from_quaternion(orientation_quat)
        angular_vel = [data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z]
        linear_accel = [data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z]
        # print("Orientation:", orientation_euler)
        # print("Angular vel:", angular_vel)
        # print("Linear accel:", linear_accel)

        self.delta_tita = self.tita_target - orientation_euler[1]
        self.delta_tita_buffer.append(self.delta_tita)
        self.delta_tita_buffer = self.delta_tita_buffer[1:]
        # self.omega = self.old_omega - 4016.9*self.delta_tita + 3485.06244*self.old_delta_tita
        self.omega = 1.32*self.omega_buffer[-1] - 0.3199*self.omega_buffer[-2] - 6069*self.delta_tita_buffer[-1] + 8900*self.delta_tita_buffer[-2] - 3034*self.delta_tita_buffer[-3]
        self.omega_buffer.append(self.omega)
        self.omega_buffer = self.omega_buffer[1:]
        # self.old_delta_tita = copy.deepcopy(self.delta_tita)
        # self.old_omega = copy.deepcopy(self.omega)

        vel = 0.012*(self.omega * self.wheel_radius)

        self.msg2pub.linear.x = vel
        self.msg2pub.angular.z = self.rotation
        self.pub.publish(self.msg2pub)
        print("Comandos --> Velocidad: {0:8.2f}  - Rotacion: {1:5.2f}".format(self.msg2pub.linear.x, self.msg2pub.angular.z))

    def joy_control(self, data):
        self.tita_target = 0.012 * data.linear.x  # Mapeo los comandos del joystick. De velocidad a angulo de inclinacion.
        self.rotation = copy.deepcopy(data.angular.z)

    def commander(self, ):
        rospy.init_node('wheel_command', anonymous=False)
        rospy.Subscriber("/joy/cmd_vel", Twist, self.joy_control)
        rospy.Subscriber("/imu", Imu, self.callback)
        # spin() simply keeps python from running until this node is stopped
        rospy.spin()


if __name__ == '__main__':
    robot_control = robot_control()
    try:
        robot_control.commander()
    except rospy.ROSInterruptException:
        pass

