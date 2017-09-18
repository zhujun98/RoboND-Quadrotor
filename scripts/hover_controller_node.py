#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
# All Rights Reserved.

# Author: Brandon Kinman
# Modified by Jun Zhu


import rospy
import tf
import math
from geometry_msgs.msg import Wrench
from geometry_msgs.msg import PoseStamped
from quad_controller.pid_controller import PIDController

from dynamic_reconfigure.server import Server as DR_Server
from quad_controller.cfg import hover_controller_paramsConfig


class HoverControllerNode(object):
    """"""
    def __init__(self):
        """Initialization"""
        self.prev_time = rospy.Time.now()

        # PID Params
        ki_max = float(rospy.get_param('~ki_max', '20'))
        kp = float(rospy.get_param('kp', '20'))
        ki = float(rospy.get_param('ki', '1'))
        kd = float(rospy.get_param('kd', '20'))

        self._controller = PIDController(kp, ki, kd, ki_max=ki_max)

        self._pose_sub = rospy.Subscriber("/quad_rotor/pose", PoseStamped, self.pose_callback)
        self._cmd_force_pub = rospy.Publisher("/quad_rotor/cmd_force", Wrench, queue_size=10)

        DR_Server(hover_controller_paramsConfig, self.config_callback)

    def config_callback(self, config, level):
        """"""
        rospy.loginfo("Re-configure request: {target}, {kp}, {ki}, {kd}".
                      format(**config))

        self._controller.set_target(config.target)
        self._controller.set_kp(config.kp)
        self._controller.set_ki(config.ki)
        self._controller.set_kd(config.kd)

        return config

    def pose_callback(self, pose_msg):
        """"""
        z_cmd = self._controller.update(pose_msg.pose.position.z,
                                        pose_msg.header.stamp.to_sec())
        cmd = Wrench()

        if z_cmd is not None:
            cmd.force.z = z_cmd
            self._cmd_force_pub.publish(cmd)


if __name__ == '__main__':
    rospy.init_node('hover_controller')
    try:
        hc = HoverControllerNode()
        rospy.spin()
    except rospy.ROSInterruptException: 
        pass