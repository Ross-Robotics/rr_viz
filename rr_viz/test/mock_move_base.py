#!/usr/bin/env python
import os
import rospy
import threading
import signal
from functools import partial
from std_srvs.srv import Trigger, TriggerResponse
from rr_node_tools_msgs.srv import String, StringResponse, StringRequest
from rr_node_tools_msgs.msg import StringArray
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult


class MockMoveBase():
    def __init__(self):
        # SlamSupervisor up is determined by existance of this service
        self._as = actionlib.SimpleActionServer(
            "/move_base", MoveBaseAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

    def execute_cb(self, _):
        return MoveBaseActionResult()


if __name__ == '__main__':
    rospy.init_node("move_base")
    m = MockMoveBase()
    rospy.spin()
