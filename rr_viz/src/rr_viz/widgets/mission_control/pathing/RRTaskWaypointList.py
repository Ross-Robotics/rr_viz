#!/usr/bin/env python
import os
import rospy
import rospkg
from interactive_waypoints.waypoint_list import InteractiveWaypointList
from rr_viz.msg import TaskWaypoint as TaskWaypointMsg
from rr_viz.msg import TaskWaypointList as TaskWaypointListMsg
from ros_msgdict import msgdict
from RRTaskWaypoint import RRTaskWaypoint


class RRTaskWaypointList(InteractiveWaypointList):

    def __init__(self):
        # Direct constructor calls.
        InteractiveWaypointList.__init__(self)

    def save_to_msg(self):
        """[Convert waypoint into a restorable ROS message. This is done for file saving convenience.]
        Returns:
            [RRTaskWaypointList]: [ros message such that output of this function given to from_msg() would recreate the RRTaskWaypointList]
        """
        msg = TaskWaypointListMsg()
        if self.len() == 0:
            return msg

        for wp in self.get_list():
            wp_msg = wp.save_to_msg()
            msg.tasklist.append(wp_msg)
        return msg

    def new_waypoint(self, msg=None):
        return RRTaskWaypoint(msg)

    def load_from_msg(self, msg):
        """[Load from RRTaskWaypointList]

        Args:
            msg ([RRTaskWaypointList]): [appropriate message to load from]
        """
        self.clearall()
        for wp_msg in msg.tasklist:
            self.append(self.new_waypoint(wp_msg))
