#!/usr/bin/env python
import os
import rospy
import rospkg
from interactive_waypoints.waypoint import Waypoint
from rr_viz.msg import TaskWaypoint
from ros_msgdict import msgdict

Waypoint.params = msgdict.yaml2msgdict(rospkg.RosPack().get_path(
    "interactive_waypoints")+"/res/waypoint_params.yaml")


class RRTaskWaypoint(Waypoint):

    def __init__(self, msg=None):
        # Direct constructor calls.
        Waypoint.__init__(self)
        self.taskSrv = "None"
        self.taskSubTree = "None"
        if msg:
            self.load_from_msg(msg)

    def set_taskSrv(self, taskSrv):
        self.taskSrv = taskSrv

    def set_taskSubTree(self, taskSubTree):
        self.taskSubTree = taskSubTree

    def get_taskSrv(self):
        return str(self.taskSrv)

    def get_taskSubTree(self):
        return str(self.taskSubTree)

    def save_to_msg(self):
        """[See Waypoint.save_to_msg]
        """
        msg = RRTaskWaypoint()
        msg.pose_stamped = self.get_pose()
        msg.tasksrv = self.get_taskSrv()
        msg.tasksubtree = self.get_taskSubTree()
        return msg

    def load_from_msg(self, msg):
        """[See Waypoint.load_from_msg]
        """
        if not isinstance(msg, RRTaskWaypoint):
            rospy.logerr("waypoint is create from RRTaskWaypoint msgs")
        self.set_pose(msg.pose_stamped)
        self.set_taskSrv(msg.tasksrv)
        self.set_taskSubTree(msg.tasksubtree)
        self._upload()
