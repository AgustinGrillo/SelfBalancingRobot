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


pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
msg2pub = Twist()
msg2pub.linear.x = 0

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


def callback(data):
    pub.publish(msg2pub)
    orientation_quat = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
    orientation_euler = euler_from_quaternion(orientation_quat)
    angular_vel = [data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z]
    linear_accel = [data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z]
    print("Orientation:", orientation_euler)
    print("Angular vel:", angular_vel)
    print("Linear accel:", linear_accel)

    if abs(orientation_euler[1]) > 0.1:
        extra = 0.5 * (orientation_euler[1]/abs(orientation_euler[1]))
    else:
        extra = 0

    msg2pub.linear.x = extra + 11*(orientation_euler[1])
    print(msg2pub.linear.x)



def commander():
    rospy.init_node('wheel_command', anonymous=False)
    rospy.Subscriber("/imu", Imu, callback)
    # spin() simply keeps python from running until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    try:
        commander()
    except rospy.ROSInterruptException:
        pass

