#!/usr/bin/env python

# Author: Devin Anzelmo

# TODO add services for setting some plot parameters to
from nav_msgs.srv import GetPlan
from nav_msgs.srv import GetPlanRequest
from nav_msgs.srv import GetPlanResponse
from nav_msgs.msg import Path

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
import numpy as np
import rospy

from std_srvs.srv import Empty
from std_srvs.srv import EmptyResponse
from std_srvs.srv import EmptyRequest
from quad_controller.srv import GetPath
from quad_controller.srv import GetPathRequest
from quad_controller.srv import SetPath
from quad_controller.srv import SetPose

from quad_controller import plotting_helpers


class QuadPlotterNode(object):

    def __init__(self):
        """Initialization"""
        self.trajectory = None
        self.waypoints = None
        self.path_history = list()
        self.running = False

        # source of data to keep track of the quads position and decide
        # when to adjust controller goals
        self.pose_sub_ = rospy.Subscriber(
            '/quad_rotor/pose', PoseStamped, self.pose_callback)

        # services for setting, and clearing the trajectory manager and
        # runs state
        self.plot_grid_srv_ = rospy.Service(
            '/quad_plotter/plot_grid', Empty, self.handle_plot_grid)
        self.plot_3d_srv_ = rospy.Service(
            '/quad_plotter/plot_3d', Empty, self.handle_plot_3d)
        self.plot_one_srv_ = rospy.Service(
            '/quad_plotter/plot_one', Empty, self.handle_plot_one)

        # services for setting, and clearing the trajectory manager and
        # runs state
        self.start_recording_srv_ = rospy.Service(
            '/quad_plotter/start_recording', Empty, self.handle_start_recording)
        self.stop_recording_srv_ = rospy.Service(
            '/quad_plotter/stop_recording', Empty, self.handle_stop_recording)
        self.clear_path_history_srv_ = rospy.Service(
            '/quad_plotter/clear_path_history', Empty, self.handle_clear_path_history)
        self.clear_waypoints_srv_ = rospy.Service(
            '/quad_plotter/clear_waypoints', Empty, self.handle_clear_waypoints)

        self.load_waypoints_from_sim_srv_ = rospy.Service(
            '/quad_plotter/load_waypoints_from_sim',
            Empty,
            self.handle_load_waypoints_from_sim)

        # services granting access to the quads history, trajectory and
        # waypoints
        self.get_path_history_srv_ = rospy.Service(
            '/quad_plotter/get_path_history', GetPath, self.handle_get_path_history)

    def handle_load_waypoints_from_sim(self, empty):
        """"""
        request = GetPathRequest()
        # rospy.wait_for_service('/quad_rotor/get_path')
        path_srv_prox_ = rospy.ServiceProxy('/quad_rotor/get_path', GetPath)
        response = path_srv_prox_(False)
        self.waypoints = path_response_to_array(response)

        return EmptyResponse()

    def handle_plot_one(self, empty):
        """"""
        xyz_arr = pose_list_to_array(self.path_history)
        plotting_helpers.plot_path_2d(
            xyz_arr, planned_path=self.trajectory, waypoints=self.waypoints)

        return EmptyResponse()

    def handle_plot_3d(self, empty):
        """"""
        xyz_arr = pose_list_to_array(self.path_history)
        plotting_helpers.plot_path_3d(
            xyz_arr, planned_path=self.trajectory, waypoints=self.waypoints)

        return EmptyResponse()

    def handle_plot_grid(self, empty):
        """"""
        xyz_arr = pose_list_to_array(self.path_history)
        plotting_helpers.plot_path_grid(
            xyz_arr, planned_path=self.trajectory, waypoints=self.waypoints)

        return EmptyResponse()

    def handle_get_path_history(self, empty):
        """"""
        if not self.path_history:
            raise RuntimeError('path history is empty, use start_recording '
                               'gather data')
        path = Path()
        path.poses = self.path_history

        return path

    def handle_start_recording(self, empty):
        """"""
        self.running = True

        return EmptyResponse()

    def handle_stop_recording(self, empty):
        """"""
        self.running = False

        return EmptyResponse()

    def handle_clear_path_history(self, empty):
        """"""
        del self.path_history
        self.path_history = list()

        return EmptyResponse()

    def handle_clear_waypoints(self, empty):
        """"""
        del self.waypoints
        self.waypoints = None

        return EmptyResponse()

    def pose_callback(self, pose):
        """"""
        if not self.running:
            return

        self.path_history.append(pose)


def path_response_to_array(get_path_response):
    """Extract x,y,z position from a GetPathResponse"""
    point_list = list()
    for pose in get_path_response.path.poses:
        arr = np.array([pose.pose.position.x,
                        pose.pose.position.y,
                        pose.pose.position.z])

        point_list.append(arr)
    return np.array(point_list)


def pose_list_to_array(poses):
    """convert poselist into an numpy 2d array"""
    tmp_list = list()
    for p in poses:
        tmp_list.append([p.pose.position.x,
                         p.pose.position.y,
                         p.pose.position.z,

                         p.pose.orientation.x,
                         p.pose.orientation.y,
                         p.pose.orientation.z,
                         p.pose.orientation.w,

                         p.header.stamp.to_sec()
                        ])

    return np.array(tmp_list)


if __name__ == "__main__":
    rospy.init_node('quad_plotter')

    try:
        rpc = QuadPlotterNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
