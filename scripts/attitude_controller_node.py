#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
# All Rights Reserved.

# Author: Brandon Kinman
# Modified by Jun Zhu


import rospy
import tf
import math
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Wrench
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64

from std_srvs.srv import Empty
from std_srvs.srv import EmptyResponse

from dynamic_reconfigure.server import Server as DR_Server
from quad_controller.cfg import attitude_controller_paramsConfig
from quad_controller.pid_controller import PIDController


class AttitudeControllerNode(object):
    """"""
    def __init__(self):
        """Initialization"""
        # Roll Controller
        roll_kp = float(rospy.get_param('~roll_kp', '0'))
        roll_ki = float(rospy.get_param('~roll_ki', '0'))
        roll_ki_max = float(rospy.get_param('~roll_ki_max', '0'))
        roll_kd = float(rospy.get_param('~roll_kd', '0'))
        self._roll_controller = PIDController(
            roll_kp, roll_ki, roll_kd, ki_max=roll_ki_max)

        # Pitch Controller
        pitch_kp = float(rospy.get_param('~pitch_kp', '0'))
        pitch_ki = float(rospy.get_param('~pitch_ki', '0'))
        pitch_ki_max = float(rospy.get_param('~pitch_ki_max', '0'))
        pitch_kd = float(rospy.get_param('~pitch_kd', '0'))
        self._pitch_controller = PIDController(
            pitch_kp, pitch_ki, pitch_kd, ki_max=pitch_ki_max)

        # Yaw controller
        yaw_kp = float(rospy.get_param('~yaw_kp', '0'))
        yaw_ki = float(rospy.get_param('~yaw_ki', '0'))
        yaw_ki_max = float(rospy.get_param('~yaw_ki_max', '0'))
        yaw_kd = float(rospy.get_param('~yaw_kd', '0'))
        self._yaw_controller = PIDController(
            yaw_kp, yaw_ki, yaw_kd, ki_max=yaw_ki_max)

        self._last_imu = Imu()
        self._z_thrust = 0.0

        self._reset_srv = rospy.Service('attitude_controller/controller_reset',
                                        Empty,
                                        self.controller_reset_callback)

        DR_Server(attitude_controller_paramsConfig, self.config_callback)

        self._imu_sub = rospy.Subscriber("/quad_rotor/imu", Imu, self.imu_callback)
        self._cmd_force_pub = rospy.Publisher(
            "/quad_rotor/cmd_force", Wrench, queue_size=10)
        self._cmd_attitude_sub = rospy.Subscriber(
            '/quad_rotor/cmd_attitude', Vector3, self.cmd_attitude_callback)
        self._cmd_z_thrust_sub = rospy.Subscriber(
            '/quad_rotor/cmd_z_thrust', Float64, self.cmd_z_thrust_callback)

    def config_callback(self, config, level):
        """"""
        rospy.loginfo('Attitude controller config changed!')

        self._roll_controller.set_kp(config.roll_kp)
        self._roll_controller.set_ki(config.roll_ki)
        self._roll_controller.set_max_ki(config.roll_ki_max)
        self._roll_controller.set_kd(config.roll_kd)

        self._pitch_controller.set_kp(config.pitch_kp)
        self._pitch_controller.set_ki(config.pitch_ki)
        self._pitch_controller.set_max_ki(config.pitch_ki_max)
        self._pitch_controller.set_kd(config.pitch_kd)

        self._yaw_controller.set_kp(config.yaw_kp)
        self._yaw_controller.set_ki(config.yaw_ki)
        self._yaw_controller.set_max_ki(config.yaw_ki_max)
        self._yaw_controller.set_kd(config.yaw_kd)

        # Dynamic re-config set point
        if config.use_dr_set_point is True:
            self._roll_controller.set_target(config.roll_set_point)
            self._pitch_controller.set_target(config.pitch_set_point)
            self._yaw_controller.set_target(config.yaw_set_point)

        return config

    def controller_reset_callback(self):
        """"""
        rospy.loginfo('Attitude controller reset!')
        self._roll_controller.reset()
        self._pitch_controller.reset()
        self._yaw_controller.reset()

        self._last_imu = Imu()
        self._z_thrust = 0.0

        return EmptyResponse()

    def cmd_attitude_callback(self, attitude_vector):
        """"""
        self._roll_controller.set_target(attitude_vector.x)
        self._pitch_controller.set_target(attitude_vector.y)
        self._yaw_controller.set_target(attitude_vector.z)

    def cmd_z_thrust_callback(self, cmd_z_thrust):
        """"""
        self._z_thrust = cmd_z_thrust.data

    def imu_callback(self, imu_msg):
        """"""
        self._last_imu = imu_msg

        roll_meas, pitch_meas, yaw_meas = \
            tf.transformations.euler_from_quaternion(
                [self._last_imu.orientation.x,
                 self._last_imu.orientation.y,
                 self._last_imu.orientation.z,
                 self._last_imu.orientation.w])

        t = self._last_imu.header.stamp.to_sec()
        roll_cmd = self._roll_controller.update(roll_meas, t)
        pitch_cmd = self._pitch_controller.update(pitch_meas, t)
        yaw_cmd = self._yaw_controller.update(yaw_meas, t)

        rospy.logdebug('roll meas: {}, pitch_meas: {}, yaw_meas: {}, '
                       'x: {}, y: {}, z: {}, w: {}'.
                       format(roll_meas, pitch_meas, yaw_meas,
                              self._last_imu.orientation.x,
                              self._last_imu.orientation.y,
                              self._last_imu.orientation.z,
                              self._last_imu.orientation.w))

        rospy.logdebug('In degrees: r:{},p:{},y:{}'.
                       format(math.degrees(roll_meas),
                              math.degrees(pitch_meas),
                              math.degrees(yaw_meas)))

        wrench_cmd = Wrench()
        wrench_cmd.torque.x = roll_cmd
        wrench_cmd.torque.y = pitch_cmd
        wrench_cmd.torque.z = yaw_cmd
        wrench_cmd.force.z = self._z_thrust

        self._cmd_force_pub.publish(wrench_cmd)


if __name__ == '__main__':
    rospy.init_node('attitude_controller')
    try:
        ac = AttitudeControllerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
