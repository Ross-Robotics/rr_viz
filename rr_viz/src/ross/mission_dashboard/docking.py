#!/usr/bin/env python
from PyQt5.QtWidgets import QVBoxLayout, QWidget, QLabel, QPushButton
from PyQt5.QtGui import QFont
from PyQt5.QtCore import Qt

import rospy
from std_srvs.srv import Trigger, TriggerRequest

class Docking(QWidget):
    def __init__(self, parent):
        super(QWidget, self).__init__(parent)

        # Setup services
        self.set_pose_srv_name = "/robot_interface/save_dock_approach"
        self.set_pose_srv = rospy.ServiceProxy(self.set_pose_srv_name, Trigger)

        self.v_layout= QVBoxLayout()

        # Title
        self.title_label = QLabel('Docking')
        self.title_label.setFont(QFont('Ubuntu', 11, QFont.Bold))
        self.title_label.setAlignment(Qt.AlignRight)
        self.v_layout.addWidget(self.title_label)

        # Button
        self.set_pose_button = QPushButton('Set Home Pose')
        self.set_pose_button.pressed.connect(self.set_pose)

        self.v_layout.addWidget(self.set_pose_button)

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
