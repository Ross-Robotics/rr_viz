#!/usr/bin/env python
import os
import rospy
import rospkg
from interactive_waypoints.waypoint import InteractiveWaypointList
from rr_viz.msg import TaskWaypoint
from ros_msgdict import msgdict
import RRTaskWaypoint


class RRTaskWaypointList(InteractiveWaypointList):

    def __init__(self):
        # Direct constructor calls.
        InteractiveWaypointList.__init__(self)

    def save_to_msg(self):
        """[Convert waypoint into a restorable ROS message. This is done for file saving convenience.]
        Returns:
            [RRTaskWaypointList]: [ros message such that output of this function given to from_msg() would recreate the RRTaskWaypointList]
        """
        msg = RRTaskWaypointList()
        if self.len() == 0:
            return msg

        for wp in self.get_list():
            wp_msg = wp.save_to_msg()
            msg.tasklist.append(wp_msg)
        return msg

    def load_from_msg(self, msg):
        """[Load from RRTaskWaypointList]

        Args:
            msg ([RRTaskWaypointList]): [appropriate message to load from]
        """
        self.clear()
        for wp_msg in msg.tasklist:
            self.append(RRTaskWaypoint(wp_msg))
