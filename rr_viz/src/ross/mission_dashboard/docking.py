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

        self.h_layout.addWidget(self.home_arm_button)
        self.h_layout.addWidget(self.set_pose_button)

        self.v_layout.addLayout(self.h_layout)

        self.setLayout(self.v_layout)

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
