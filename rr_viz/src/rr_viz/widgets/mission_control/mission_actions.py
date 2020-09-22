#!/usr/bin/env python
import rospy
import actionlib
import os
from rr_viz.srv import BuildBT, BuildBTRequest
from interactive_waypoints.waypoint_actions import on_own_thread


class ServiceAction(object):

    def __init__(self, srv_name, srvType):
        self.srv_name = srv_name
        self._srv_connected = False
        # Timer for monitoring conenction
        self.srv_timer = rospy.Timer(rospy.Duration(
            5), lambda event: self.ping_srv)
        self._srv_client = rospy.ServiceProxy(
            srv_name, srvType)

    def ping_srv(self):
        if not self._srv_client.wait_for_service(timeout=rospy.Duration(3.0)):
            rospy.loginfo_throttle(30,
                                   "Lost Connection to {}".format(self.srv_name) if self._srv_connected else "Failed to connect to movebase")
        else:
            if not self._srv_connected:
                rospy.loginfo('Connected to {}.'.format(self.srv_name))
            self._srv_connected = True

    def is_connected(self):
        return self._srv_connected


class BuildBTAction(ServiceAction):

    def __init__(self):
        ServiceAction.__init__(self, "build_bt", BuildBT)

    @on_own_thread
    def build_bt_action(self, waypoint_list):
        btbr = BuildBTRequest()
        btbr.name = "Mission" + \
            str(rospy.Time.now())  # Makes every wp unique
        rrtasklist = waypoint_list.save_to_msg()
        btbr.tasklist = rrtasklist.tasklist
        resp = self._srv_client(btbr)
        if not resp.success:
            rospy.logerr("bt building failed")
