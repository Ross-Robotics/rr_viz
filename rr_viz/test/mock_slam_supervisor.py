#!/usr/bin/env python
import os
import rospy
import threading
import signal
from functools import partial
from std_srvs.srv import Trigger, TriggerResponse
from rr_node_tools_msgs.srv import String, StringResponse, StringRequest
from rr_node_tools_msgs.msg import StringArray


class MockSlamSupervisor():
    def __init__(self):
        # SlamSupervisor up is determined by existance of this service
        self.kill_nodes = rospy.Service(
            '~kill_nodes', Trigger, self.handle_trigger)
        self.kill_nodes = rospy.Service(
            '~launch_mapping', Trigger, self.handle_trigger)
        self.kill_nodes = rospy.Service(
            '~launch_localization', String, self.handle_string)
        self.kill_nodes = rospy.Service(
            '~list_maps', Trigger, self.handle_trigger)
        self.kill_nodes = rospy.Service(
            '~save_map', String, self.handle_string)

    def handle_trigger(self, _):
        return TriggerResponse()

    def handle_string(self, _):
        return StringResponse()


if __name__ == '__main__':
    rospy.init_node("slam_supervisor")
    m = MockSlamSupervisor()
    rospy.spin()
