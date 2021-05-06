#!/usr/bin/env python
import os
import rospy
import rospkg
from interactive_waypoints.waypoint import Waypoint
from interactive_waypoints import waypoint
from rr_custom_msgs.msg import TaskWaypoint as TaskWaypointMsg
from ros_msgdict import msgdict

params = msgdict.yaml2msgdict(rospkg.RosPack().get_path(
    "rr_viz")+"/res/waypoint_params.yaml")
waypoint.params = params


class RRTaskWaypoint(Waypoint):

    def __init__(self, msg=None):
        # Direct constructor calls.
        Waypoint.__init__(self)
        self.taskSrv = "None"
        self.taskSubTree = "None"
        if msg:
            self.load_from_msg(msg)

    def duplicate(self):
        """[Create a new waypoint using the data of this waypoint. e.g. Duplicate]

        Returns:
            [type]: [description]
        """
        self.__class__(self.save_to_msg())
        return self.__class__(self.save_to_msg())

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
        msg = TaskWaypointMsg()
        msg.pose_stamped = self.get_pose()
        msg.tasksrv = self.get_taskSrv()
        msg.tasksubtree = self.get_taskSubTree()
        return msg

    def load_from_msg(self, msg):
        """[See Waypoint.load_from_msg]
        """
        if not isinstance(msg, TaskWaypointMsg):
            rospy.logerr("waypoint is create from RRTaskWaypoint msgs")
        self.set_pose(msg.pose_stamped)
        self.set_taskSrv(msg.tasksrv)
        self.set_taskSubTree(msg.tasksubtree)
        self._upload()
