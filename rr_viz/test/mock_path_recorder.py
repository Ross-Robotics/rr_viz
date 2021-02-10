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
import actionlib
from rr_custom_msgs.srv import String, StringResponse, StringRequest
from rr_custom_msgs.msg import StringArray
from rr_custom_msgs.msg import RecordPathAction, RecordPathResult, RecordPathFeedback
from std_msgs.msg import String


def randomString(stringLength):
    letters = string.ascii_letters
    return ''.join(random.choice(letters) for i in range(stringLength))


class MockPathRecorder():
    def __init__(self):
        self.finish_req_serv = rospy.Service(
            "~finish_path", Trigger, self.finish_path_cb)
        self._as = actionlib.SimpleActionServer(
            "~record", RecordPathAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
        self.status_pub = rospy.Publisher("~status", String, queue_size=10)
        self.status_pub_timer = rospy.Timer(rospy.Duration(1.), lambda _: self.status_pub.publish(
            "Recording" if self._as.is_active() else "Not Recording"))

    def execute_cb(self, goal):
        ret = RecordPathResult()
        feedback = RecordPathFeedback()
        self.r = rospy.Rate(1)
        while not rospy.is_shutdown():
            if self._as.is_preempt_requested() or self.finish_requested:
                rospy.loginfo("action record was cancelled")
                self._as.set_succeeded(ret)
                self.finish_requested = False
                return ret
            self._as.publish_feedback(feedback)
            self.r.sleep()

    def finish_path_cb(self, srv):
        # Set finsih flag only if there is a goal running.
        rospy.loginfo("finish_path service called")
        if self._as.is_active():
            self.finish_requested = True
        return TriggerResponse(success=True, message="Finish set")


if __name__ == '__main__':
    rospy.init_node("path_recorder")
    m = MockPathRecorder()
    rospy.spin()



