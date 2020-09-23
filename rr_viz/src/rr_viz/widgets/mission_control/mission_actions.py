#!/usr/bin/env python
import rospy
import actionlib
import os
from rr_viz.srv import BuildBT, BuildBTRequest
from interactive_waypoints.waypoint_actions import on_own_thread, _movebase_command
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class ServiceAction(object):

    def __init__(self, srv_name, srvType):
        self.srv_name = srv_name
        self._srv_connected = False
        # Timer for monitoring conenction
        self._srv_client = rospy.ServiceProxy(
            srv_name, srvType)
        self.srv_timer = rospy.Timer(rospy.Duration(
            5), lambda event: self.ping_srv())

    def ping_srv(self):
        if rospy.is_shutdown():
            return False
        try:
            self._srv_client.wait_for_service(timeout=rospy.Duration(3.0))
            if not self._srv_connected:
                rospy.loginfo('Connected to {}.'.format(self.srv_name))
            self._srv_connected = True
        except:
            rospy.loginfo_throttle(30,
                                   "Lost Connection to {}".format(self.srv_name) if self._srv_connected else "Failed to connect to {}".format(self.srv_name))
            self._srv_connected = False

    def is_connected(self):
        return self._srv_connected


class ActionServerAction(object):

    def __init__(self, action_name, actionType):
        self.act_name = action_name
        self._act_connected = False
        # Timer for monitoring conenction
        self._act_client = actionlib.SimpleActionClient(
            action_name, actionType)
        self._act_timer = rospy.Timer(rospy.Duration(
            5), lambda event: self.ping_act())

    def ping_act(self):
        if rospy.is_shutdown():
            return False
        if not self._act_client.wait_for_server(timeout=rospy.Duration(3.0)):
            rospy.loginfo_throttle(30,
                                   "Waypoint lost connection to {}".format(self.act_name) if self._act_connected else "Failed to connect to {}".format(self.act_name))
        else:
            if not self._act_connected:
                rospy.loginfo(
                    "Waypoint follower Connected to {}".format(self.act_name))
            self._act_connected = True

    def is_connected(self):
        return self._act_connected


class WaypointMoveBaseAction(ActionServerAction):

    def __init__(self):
        ActionServerAction.__init__(self, rospy.get_param(
            "move_base_namespace", "/move_base"), MoveBaseAction)

    @_movebase_command
    def _send_goal_pose(self, pose):
        goal = MoveBaseGoal()
        goal.target_pose = pose
        self._act_client.send_goal(goal)
        self._act_client.wait_for_result()

    @_movebase_command
    def cancel_goals_action(self):
        self._act_client.cancel_all_goals()

    @on_own_thread
    def goto_action(self, wp_list):
        wp = wp_list.get_selected_wp()
        if not wp:
            rospy.logerr("No wp selected")
            return
        if self._act_connected:
            self._send_goal_pose(wp.get_pose())
        else:
            rospy.logerr(
                "Attempting to invoke move_base, but client is not connected")

    @on_own_thread
    def gotoall_action(self, wp_list):
        msg = wp_list.save_to_msg()
        if self._act_connected:
            for task in msg.tasklist:
                self._send_goal_pose(task.pose_stamped)
        else:
            rospy.logerr(
                "Attempting to invoke move_base, but client is not connected")


class BuildBTAction(ServiceAction):

    def __init__(self):
        ServiceAction.__init__(self, rospy.get_param(
            "build_bt_srv", "/build_bt"), BuildBT)

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
