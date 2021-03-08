#!/usr/bin/env python
import os
import rospy
import threading
import signal
from functools import partial
from std_srvs.srv import Trigger, TriggerResponse
from rr_custom_msgs.msg import StringArray
from rr_custom_msgs.srv import String, StringResponse, StringRequest
from rr_custom_msgs.srv import BuildBT, BuildBTResponse


class MockBTBuilder():
    def __init__(self):
        # SlamSupervisor up is determined by existance of this service
        self.build_bt = rospy.Service(
            '/build_bt', BuildBT, self.handle_bt)

    def handle_bt(self, _):
        rospy.logwarn("BT BUILT")
        return BuildBTResponse()


if __name__ == '__main__':
    rospy.init_node("bt_builder")
    m = MockBTBuilder()
    rospy.spin()
