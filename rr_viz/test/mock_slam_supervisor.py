#!/usr/bin/env python
import os
import rospy
import threading
import signal
from functools import partial
from std_srvs.srv import Trigger


class MockSlamSupervisor():
    def __init__(self):
        # SlamSupervisor up is determined by existance of this service
        self.s = rospy.Service('kill_nodes', Trigger, self.handle)

    def handle(self, _):
        pass


if __name__ == '__main__':
    rospy.init_node("slam_supervisor")
    m = MockSlamSupervisor()
    rospy.spin()
