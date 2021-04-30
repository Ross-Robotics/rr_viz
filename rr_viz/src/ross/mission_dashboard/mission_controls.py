#!/usr/bin/env python
from PyQt5.QtWidgets import QVBoxLayout, QHBoxLayout, QWidget, QLabel, QPushButton, QGridLayout
from PyQt5.QtGui import QFont
from PyQt5.QtCore import Qt

import rospy
from std_srvs.srv import Trigger, TriggerRequest
from rr_custom_msgs.srv import SetMode
from rr_custom_msgs.msg import Mode
from std_msgs.msg import String
import string

MAX_LABEL_CHAR_WIDTH = 30

class MissionControls(QWidget):
    def __init__(self, parent):
        super(QWidget, self).__init__(parent)

        # Setup services
        self.start_mission_srv_name = "/robot_interface/start_mission"
        self.next_waypoint_srv_name = "/robot_interface/continue_mission"
        self.stop_mission_srv_name = "/robot_interface/stop_mission"
        self.set_active_mode_srv_name = "/robot_interface/set_robot_mode"

        self.start_mission_srv = rospy.ServiceProxy(self.start_mission_srv_name, Trigger)
        self.next_waypoint_srv = rospy.ServiceProxy(self.next_waypoint_srv_name, Trigger)
        self.stop_mission_srv = rospy.ServiceProxy(self.stop_mission_srv_name, Trigger)
        self.set_active_mode_srv = rospy.ServiceProxy(self.set_active_mode_srv_name, SetMode)

        # Setup topic subscribers
        self.status_label_topic_name = "/robot_interface/mission_status"
        self.status_label_sub = rospy.Subscriber(self.status_label_topic_name, String, self.status_label_update)

        self.v_layout = QVBoxLayout()

        # Title
        self.title_label = QLabel('Mission Controls')
        self.title_label.setFont(QFont('Ubuntu', 11, QFont.Bold))
        self.title_label.setAlignment(Qt.AlignRight)
        self.v_layout.addWidget(self.title_label)

        # Status
        self.h_layout = QHBoxLayout()
        self.status_text_label = QLabel('Status:')
        self.status_text_label.setFont(QFont('Ubuntu', 10, QFont.Bold))
        self.status_label = QLabel('N/A')
        self.status_label.setWordWrap(True)
        self.h_layout.addWidget(self.status_text_label, 2)
        self.h_layout.addWidget(self.status_label, 8)
        
        self.v_layout.addLayout(self.h_layout)

        # Buttons
        self.grid_layout = QGridLayout()

        self.start_mission_button = QPushButton('Start Mission')
        self.start_mission_button.pressed.connect(self.start_mission)

        self.next_waypoint_button = QPushButton('Next Waypoint')
        self.next_waypoint_button.pressed.connect(self.next_waypoint)

        self.stop_mission_button = QPushButton('Stop Mission')
        self.stop_mission_button.pressed.connect(self.stop_mission)

        self.set_active_button = QPushButton('Set Active')
        self.set_active_button.pressed.connect(self.set_active_mode)

        self.grid_layout.addWidget(self.start_mission_button, 0, 1)
        self.grid_layout.addWidget(self.next_waypoint_button, 1, 0)
        self.grid_layout.addWidget(self.stop_mission_button, 1, 1)
        self.grid_layout.addWidget(self.set_active_button, 0, 0)

        self.v_layout.addLayout(self.grid_layout)

        self.setLayout(self.v_layout)

    def start_mission(self):
        try:
            trig_resp = self.start_mission_srv.call(TriggerRequest())
            print(trig_resp.message)
            if not trig_resp.success:
                msg = "Failed to call '" + self.start_mission_srv_name + "' service"
                rospy.logerr(msg)
        except:
            msg = "Service '" + self.start_mission_srv_name + "' unavailable"
            rospy.logwarn(msg)

    def next_waypoint(self):
        try:
            trig_resp = self.next_waypoint_srv.call(TriggerRequest())
            print(trig_resp.message)
            if not trig_resp.success:
                msg = "Failed to call '" + self.next_waypoint_srv_name + "' service"
                rospy.logerr(msg)
        except:
            msg = "Service '" + self.next_waypoint_srv_name + "' unavailable"
            rospy.logwarn(msg)

    def stop_mission(self):
        try:
            trig_resp = self.stop_mission_srv.call(TriggerRequest())
            print(trig_resp.message)
            if not trig_resp.success:
                msg = "Failed to call '" + self.stop_mission_srv_name + "' service"
                rospy.logerr(msg)
        except:
            msg = "Service '" + self.stop_mission_srv_name + "' unavailable"
            rospy.logwarn(msg)

    def set_active_mode(self):
        mode = Mode()
        mode.mode = mode.ACTIVE

        try:
            trig_resp = self.set_active_mode_srv.call(mode)
            print(trig_resp.message)
            if not trig_resp.success:
                msg = "Failed to call '" + self.set_active_mode_srv_name + "' service"
                rospy.logerr(msg)
        except:
            msg = "Service '" + self.set_active_mode_srv_name + "' unavailable"
            rospy.logwarn(msg)

    def status_label_update(self, msg):
        if(len(msg.data) > MAX_LABEL_CHAR_WIDTH):
            text_to_display = string.replace(msg.data, ',', ',\n')
        else:
            text_to_display = msg.data

        self.status_label.setText(text_to_display)
