#!/usr/bin/env python
import os
import rospy
import threading
import signal
import random
import string
from functools import partial
from std_srvs.srv import Trigger, TriggerResponse
import std_msgs.msg
from rr_node_tools_msgs.srv import String, StringResponse, StringRequest
from rr_node_tools_msgs.msg import StringArray


def randomString(stringLength):
    letters = string.ascii_letters
    return ''.join(random.choice(letters) for i in range(stringLength))


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
            '~list_maps', Trigger, self.handle_map_list_trigger)
        self.kill_nodes = rospy.Service(
            '~save_map', String, self.handle_string)
        self.mode_pub = rospy.Publisher("~mode", std_msgs.msg.String, queue_size=10)
        self.mode_timer = rospy.Timer(rospy.Duration(
            1.0), lambda _: self.mode_pub.publish(randomString(5)))        

    def handle_trigger(self, _):
        return TriggerResponse()

    def handle_map_list_trigger(self, _):
        mock_files = ["tomato.posegraph", "potato.yaml", "pomato.yaml",
                      "potato.posegraph", "pomato.posegraph", "tomato.yaml"]
        file_list_str = ",".join(mock_files)
        return TriggerResponse(success=True, message=file_list_str)

    def handle_string(self, _):
        return StringResponse()


if __name__ == '__main__':
    rospy.init_node("slam_supervisor")
    m = MockSlamSupervisor()
    rospy.spin()
