#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
# All Rights Reserved.

# Author: Brandon Kinman
# Modified by Jun Zhu


import rospy
import tf

from geometry_msgs.msg import PoseStamped
from quad_controller.msg import EulerAngles
from sensor_msgs.msg import Imu


class QuaternionToEuler(object):
    """"""
    def __init__(self):
        """Initialization"""
        self._imu_sub = rospy.Subscriber(
            "/quad_rotor/imu", Imu, self.imu_callback)
        self._pose_sub = rospy.Subscriber(
            "/quad_rotor/pose", PoseStamped, self.pose_callback)
        self._imu_euler_pub = rospy.Publisher(
            "/quad_rotor/imu_euler_angles", EulerAngles, queue_size=1)
        self._pose_euler_pub = rospy.Publisher(
            "/quad_rotor/pose_euler_angles", EulerAngles, queue_size=1)

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()
    
    def imu_callback(self, msg):
        """"""
        ea_msg = EulerAngles()
        ea_msg.header = msg.header
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
            [msg.orientation.x, msg.orientation.y,
             msg.orientation.z, msg.orientation.w])

        ea_msg.roll = roll
        ea_msg.pitch = pitch
        ea_msg.yaw = yaw

        self._imu_euler_pub.publish(ea_msg)

    def pose_callback(self, msg):
        """"""
        ea_msg = EulerAngles()
        ea_msg.header = msg.header

        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
            [msg.pose.orientation.x, msg.pose.orientation.y,
             msg.pose.orientation.z, msg.pose.orientation.w])

        ea_msg.roll = roll
        ea_msg.pitch = pitch
        ea_msg.yaw = yaw

        self._pose_euler_pub.publish(ea_msg)


if __name__ == '__main__':
    rospy.init_node('quaternion_to_euler')
    
    try:
        q2e = QuaternionToEuler()
    except rospy.ROSInterruptException:
        pass