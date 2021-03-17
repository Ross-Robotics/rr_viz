#!/usr/bin/env python
import sys
from PyQt5.QtWidgets import QVBoxLayout, QHBoxLayout, QWidget, QLabel, QPushButton
from PyQt5.QtGui import QFont
from PyQt5.QtCore import Qt

import rospy
from std_srvs.srv import Trigger, TriggerRequest

class Docking(QWidget):
    def __init__(self, parent):
        super(QWidget, self).__init__(parent)

        # Setup services
        self.set_pose_srv_name = "/robot_interface/save_dock_approach"
        self.go_to_dock_srv_name = "/robot_interface/go_to_base"
        self.set_pose_srv = rospy.ServiceProxy(self.set_pose_srv_name, Trigger)
        self.go_to_dock_srv = rospy.ServiceProxy(self.go_to_dock_srv_name, Trigger)

        self.v_layout= QVBoxLayout()

        # Title
        self.title_label = QLabel('Docking')
        self.title_label.setFont(QFont('Ubuntu', 11, QFont.Bold))
        self.title_label.setAlignment(Qt.AlignRight)
        self.v_layout.addWidget(self.title_label)

        # Buttons
        self.h_layout = QHBoxLayout()

        self.set_pose_button = QPushButton('Set Docking Pose')
        self.set_pose_button.pressed.connect(self.set_pose)

        self.go_to_dock_button = QPushButton('Go to Dock')
        self.go_to_dock_button.pressed.connect(self.go_to_dock)

        self.h_layout.addWidget(self.set_pose_button)
        self.h_layout.addWidget(self.go_to_dock_button)
        self.v_layout.addLayout(self.h_layout)

        self.setLayout(self.v_layout)

    def set_pose(self):
        try:
            trig_resp = self.set_pose_srv.call(TriggerRequest())
            if trig_resp.success:
                print(trig_resp.message)
            else:
                print(trig_resp.message)
                msg = "Failed to call '" + self.set_pose_srv_name + "' service"
                print(msg)
        except:
            msg = "Service '" + self.set_pose_srv_name + "' unavailable"
            rospy.logwarn(msg)


    def go_to_dock(self):
        try:
            trig_resp = self.go_to_dock_srv.call(TriggerRequest())
            if trig_resp.success:
                print(trig_resp.message)
            else:
                msg = "Failed to call '" + self.go_to_dock_srv_name + "' service"
                print(msg)
        except:
            msg = "Service '" + self.go_to_dock_srv_name + "' unavailable"
            rospy.logwarn(msg)