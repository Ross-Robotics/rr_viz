#!/usr/bin/env python
import os
import rospy
import threading
import signal
from functools import partial
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import String
import random
import string


class MockEnvirobotInterface():
    def __init__(self):
        services = ["start_mission", "stop_mission", "cancel_emergency_stop",
                    "emergency_stop", "save_dock_approach", "go_to_base"]
        self.services = []
        for service in services:
            self.services.append(rospy.Service(
                "~"+service, Trigger, lambda request,service=service: self.handle_trigger(service, request)))
        self.pub = rospy.Publisher("~mission_status", String, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(
            1.0 / 2), lambda _: self.pub.publish(''.join(random.choice(string.ascii_lowercase))))

    def handle_trigger(self, service_name, request):
        rospy.loginfo("Service " + service_name + " was called")
        return TriggerResponse()


if __name__ == '__main__':
    rospy.init_node("robot_interface")
    m = MockEnvirobotInterface()
    rospy.spin()
