#!/usr/bin/env python
from PyQt5.QtWidgets import QVBoxLayout, QWidget, QLabel, QPushButton, QHBoxLayout
from PyQt5.QtGui import QFont
from PyQt5.QtCore import Qt

import actionlib
import rospy
from std_srvs.srv import Trigger, TriggerRequest
from rr_custom_msgs.msg import GoToHomeAction, GoToHomeGoal

class Docking(QWidget):
    def __init__(self, parent):
        super(QWidget, self).__init__(parent)

        # Setup services
        self.set_pose_srv_name = "/robot_interface/save_dock_approach"
        self.set_pose_srv = rospy.ServiceProxy(self.set_pose_srv_name, Trigger)
        self.home_arm_action_name = "/mk3_g_arm/mk3_g_arm/go_to_home"
        self.home_arm_action = actionlib.SimpleActionClient(self.home_arm_action_name, GoToHomeAction)
        self.lights_srv_name = "/light_interface/trigger_led"
        self.lights_srv = rospy.ServiceProxy(self.lights_srv_name, Trigger)  

        self.v_layout = QVBoxLayout()
        self.h_layout = QHBoxLayout()

        # Title
        self.title_label = QLabel('Docking')
        self.title_label.setFont(QFont('Ubuntu', 11, QFont.Bold))
        self.title_label.setAlignment(Qt.AlignRight)
        self.v_layout.addWidget(self.title_label)

        # Buttons
        self.home_arm_button = QPushButton('Home Arm')
        self.home_arm_button.pressed.connect(self.home_arm)

        self.set_pose_button = QPushButton('Set Home Pose')
        self.set_pose_button.pressed.connect(self.set_pose)

        self.lights_btn = QPushButton('Toggle Lights')
        self.lights_btn.pressed.connect(self.lights)

        self.enable_lights_btn(False)

        self.h_layout.addWidget(self.home_arm_button)
        self.h_layout.addWidget(self.set_pose_button)
        self.h_layout.addWidget(self.lights_btn)

        self.v_layout.addLayout(self.h_layout)

        self.setLayout(self.v_layout)

        # Setup service checker
        self.lights_srv_timer = rospy.Timer(rospy.Duration(
            2), self.ping_srv)
        self.lights_srv_connected = False

    def set_pose(self):
        try:
            trig_resp = self.set_pose_srv.call(TriggerRequest())
            rospy.loginfo(trig_resp.message)
            if not trig_resp.success:
                msg = "Failed to call '" + self.set_pose_srv_name + "' service"
                rospy.logerr(msg)
        except:
            msg = "Service '" + self.set_pose_srv_name + "' unavailable"
            rospy.logwarn(msg)

    def home_arm(self):
        self.home_arm_action.send_goal(GoToHomeGoal(go_to_home=True))

    def lights(self):
        try:
            trig_resp = self.lights_srv.call(TriggerRequest())
            rospy.loginfo(trig_resp.message)
            if not trig_resp.success:
                msg = "Failed to call '" + self.lights_srv_name + "' service"
                rospy.logerr(msg)
        except:
            msg = "Service '" + self.lights_srv_name + "' unavailable"
            rospy.logwarn(msg)

    def enable_lights_btn(self, enabled):
        self.lights_btn.setEnabled(enabled)

    def ping_srv(self, period):
        if rospy.is_shutdown():
            return False
        try:
            self.lights_srv.wait_for_service(timeout=rospy.Duration(3.0))
            if not self.lights_srv_connected:
                rospy.loginfo_throttle(1,'Connected to {}.'.format(self.lights_srv_name))
            self.lights_srv_connected = True
            self.enable_lights_btn(True)
            return True
        except:
            rospy.loginfo_throttle(1,
                                   "Lost Connection to {}".format(self.lights_srv_name) if self.lights_srv_connected else "Failed to connect to {}".format(self.lights_srv_name))
            self.lights_srv_connected = False
            self.enable_lights_btn(False)

        
