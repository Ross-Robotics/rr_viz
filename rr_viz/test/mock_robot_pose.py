#!/usr/bin/env python
import os
import rospy
import threading
import signal
from functools import partial
from std_srvs.srv import Trigger, TriggerResponse
from geometry_msgs.msg import PoseStamped
import random


class MockRobotpose():
    def __init__(self):
        # SlamSupervisor up is determined by existance of this service
        self.pub = rospy.Publisher("/robot_pose", PoseStamped, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(
            1.0 / 10), lambda _: self.pub.publish(self.rand_pose()))

    def rand_pose(self):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = random.uniform(0, 5)
        return pose


if __name__ == '__main__':
    rospy.init_node("bt_builder")
    m = MockRobotpose()
    rospy.spin()
