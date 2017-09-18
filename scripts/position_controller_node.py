#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
# All Rights Reserved.

# Author: Brandon Kinman
# Modified by Jun Zhu


import rospy
import tf
import math
import numpy as np
from geometry_msgs.msg import Wrench
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from quad_controller.pid_controller import PIDController

from std_srvs.srv import SetBool
from std_srvs.srv import SetBoolResponse

from std_srvs.srv import Empty
from std_srvs.srv import EmptyResponse

from dynamic_reconfigure.server import Server as DR_Server
from quad_controller.cfg import position_controller_paramsConfig


class PositionControllerNode(object):
    """"""
    def __init__(self):
        """Initialization"""
        self._first_pose_received = False
        self._counter = 0

        # X PID Config
        x_kp = float(rospy.get_param('x_kp', '0'))
        x_ki = float(rospy.get_param('x_ki', '0'))
        x_ki_max = float(rospy.get_param('x_ki_max', '0'))
        x_kd = float(rospy.get_param('x_kd', '0'))
        self._x_controller = PIDController(x_kp, x_ki, x_kd, ki_max=x_ki_max)

        # Y PID Config
        y_kp = float(rospy.get_param('y_kp', '0'))
        y_ki = float(rospy.get_param('y_ki', '0'))
        y_ki_max = float(rospy.get_param('y_ki_max', '0'))
        y_kd = float(rospy.get_param('y_kd', '0'))
        self._y_controller = PIDController(y_kp, y_ki, y_kd, ki_max=y_ki_max)

        # Z PID Config
        z_kp = float(rospy.get_param('z_kp', '0'))
        z_ki = float(rospy.get_param('z_ki', '0'))
        z_ki_max = float(rospy.get_param('z_ki_max', '0'))
        z_kd = float(rospy.get_param('z_kd', '0'))
        self._z_controller = PIDController(z_kp, z_ki, z_kd, ki_max=z_ki_max)

        self._pose_sub = rospy.Subscriber(
            '/quad_rotor/pose', PoseStamped, self.pose_callback)

        self._reset_srv = rospy.Service('/position_controller/controller_reset',
                                        Empty,
                                        self.controller_reset_callback)

        self._cmd_z_thrust_pub = rospy.Publisher(
            '/quad_rotor/cmd_z_thrust', Float64, queue_size=10)
        self._cmd_attitude_pub = rospy.Publisher(
            '/quad_rotor/cmd_attitude', Vector3, queue_size=10)

        DR_Server(position_controller_paramsConfig, self.config_callback)

    def config_callback(self, config, level):
        """"""
        rospy.loginfo('Position controller node config changed!')

        # X Config Params
        self._x_controller.set_kp(config.x_kp)
        self._x_controller.set_ki(config.x_ki)
        self._x_controller.set_max_ki(config.x_ki_max)
        self._x_controller.set_kd(config.x_kd)

        # Y Config Params
        self._y_controller.set_kp(config.y_kp)
        self._y_controller.set_ki(config.y_ki)
        self._y_controller.set_max_ki(config.y_ki_max)
        self._y_controller.set_kd(config.y_kd)

        # Z Config Params
        self._z_controller.set_kp(config.z_kp)
        self._z_controller.set_ki(config.z_ki)
        self._z_controller.set_max_ki(config.z_ki_max)
        self._z_controller.set_kd(config.z_kd)

        # Dynamic re-config set-point
        if config.use_dr_set_point:
            self._x_controller.set_target(config.set_point_x)
            self._y_controller.set_target(config.set_point_y)
            self._z_controller.set_target(config.set_point_z)

        return config

    def controller_reset_callback(self, controller_reset_msg):
        """"""
        rospy.loginfo('Position controller reset!')
        self._x_controller.reset()
        self._y_controller.reset()
        self._z_controller.reset()
        self._first_pose_received = False

        return EmptyResponse()

    def pose_callback(self, pose_msg):
        """"""
        # 10 is the ratio between the frequencies of the outer loop
        # and the inner loop.
        if self._counter >= 10 or not self._first_pose_received:
            self._counter = 0

            # Initial goal should be starting position.
            if self._first_pose_received is False:
                self._first_pose_received = True
                self._x_controller.set_target(pose_msg.pose.position.x)
                self._y_controller.set_target(pose_msg.pose.position.y)
                self._z_controller.set_target(pose_msg.pose.position.z)

            t = pose_msg.header.stamp.to_sec()

            # Control Roll to to move along Y
            roll = -(1/9.81) * self._y_controller.update(
                pose_msg.pose.position.y, t)

            # Control Pitch to move along X
            pitch = (1/9.81) * self._x_controller.update(
                pose_msg.pose.position.x, t)

            # Control Thrust to move along Z
            z_thrust = self._z_controller.update(pose_msg.pose.position.z, t)

            roll = min(max(roll, -15*(math.pi/180)), 15*(math.pi/180))
            pitch = min(max(pitch, -15*(math.pi/180)), 15*(math.pi/180))

            rotation_cmd = Vector3()
            z_thrust_cmd = Float64()

            rotation_cmd.x = roll
            rotation_cmd.y = pitch

            # Compensate for g and clip thrust
            z_thrust_cmd.data = min(max(z_thrust + 2*9.81, -30 + 2*9.81), 30 + 2*9.81)

            rospy.logdebug('roll:{}, pitch:{}, z_thrust:{}'.format(roll, pitch, z_thrust))
            self._cmd_attitude_pub.publish(rotation_cmd)
            self._cmd_z_thrust_pub.publish(z_thrust_cmd)
        else:
            self._counter += 1


if __name__ == '__main__':
    rospy.init_node('position_controller')
    try:
        ac = PositionControllerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
