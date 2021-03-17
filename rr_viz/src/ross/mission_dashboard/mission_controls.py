#!/usr/bin/env python
import sys
from PyQt5.QtWidgets import QVBoxLayout, QHBoxLayout, QWidget, QLabel, QPushButton, QGridLayout
from PyQt5.QtGui import QFont
from PyQt5.QtCore import Qt

import rospy
from std_srvs.srv import Trigger, TriggerRequest
from rr_custom_msgs.srv import SetMode, SetModeRequest, SetModeResponse
from rr_custom_msgs.msg import Mode
from std_msgs.msg import String

class MissionControls(QWidget):
    def __init__(self, parent):
        super(QWidget, self).__init__(parent)

        # Setup services
        self.start_mission_srv_name = "/robot_interface/start_mission"
        self.next_waypoint_srv_name = "/robot_interface/continue_mission"
        self.stop_mission_srv_name = "/robot_interface/stop_mission"
        self.emergency_stop_srv_name = "/robot_interface/emergency_stop"
        self.emergency_stop_resume_srv_name = "/robot_interface/cancel_emergency_stop"
        self.set_active_mode_srv_name = "/robot_interface/set_robot_mode"

        self.start_mission_srv = rospy.ServiceProxy(self.start_mission_srv_name, Trigger)
        self.next_waypoint_srv = rospy.ServiceProxy(self.next_waypoint_srv_name, Trigger)
        self.stop_mission_srv = rospy.ServiceProxy(self.stop_mission_srv_name, Trigger)
        self.emergency_stop_srv = rospy.ServiceProxy(self.emergency_stop_srv_name, Trigger)
        self.emergency_stop_resume_srv = rospy.ServiceProxy(self.emergency_stop_resume_srv_name, Trigger)
        self.set_active_mode_srv = rospy.ServiceProxy(self.set_active_mode_srv_name, SetMode)

        # Setup topic subscribers
        self.status_label_topic_name = "/robot_interface/mission_status"
        self.status_label_sub = rospy.Subscriber(self.status_label_topic_name, String, self.status_label_update)

        self.v_layout = QVBoxLayout()

        #Title
        self.title_label = QLabel('Mission Controls')
        self.title_label.setFont(QFont('Ubuntu', 11, QFont.Bold))
        self.title_label.setAlignment(Qt.AlignRight)
        self.v_layout.addWidget(self.title_label)

        #Status
        self.h_layout = QHBoxLayout()
        self.status_text_label = QLabel('Status:')
        self.status_text_label.setFont(QFont('Ubuntu', 10, QFont.Bold))
        self.status_label = QLabel('N/A')
        self.h_layout.addWidget(self.status_text_label, 2)
        self.h_layout.addWidget(self.status_label, 8)
        
        self.v_layout.addLayout(self.h_layout)

        #buttons
        self.grid_layout = QGridLayout()

        self.start_mission_button = QPushButton('Start Mission')
        self.start_mission_button.pressed.connect(self.start_mission)

        self.next_waypoint_button = QPushButton('Next Waypoint')
        self.next_waypoint_button.pressed.connect(self.next_waypoint)

        self.stop_mission_button = QPushButton('Stop Mission')
        self.stop_mission_button.pressed.connect(self.stop_mission)

        self.set_active_button = QPushButton('Set Active')
        self.set_active_button.pressed.connect(self.set_active_mode)

        self.emergency_stop_button = QPushButton('EMERGENCY STOP')
        self.emergency_stop_button.pressed.connect(self.emergency_stop)

        self.emergency_stop_resume_button = QPushButton('eStop Resume')
        self.emergency_stop_resume_button.pressed.connect(self.emergency_stop_resume)

        self.grid_layout.addWidget(self.start_mission_button, 0, 0)
        self.grid_layout.addWidget(self.next_waypoint_button, 0, 1)
        self.grid_layout.addWidget(self.stop_mission_button, 0, 2)
        self.grid_layout.addWidget(self.set_active_button, 1, 0)
        self.grid_layout.addWidget(self.emergency_stop_button, 1, 1)
        self.grid_layout.addWidget(self.emergency_stop_resume_button, 1, 2)

        self.v_layout.addLayout(self.grid_layout)

        self.setLayout(self.v_layout)

    def start_mission(self):
        try:
            trig_resp = self.start_mission_srv.call(TriggerRequest())
            if trig_resp.success:
                print(trig_resp.message)
            else:
                print(trig_resp.message)
                msg = "Failed to call '" + self.start_mission_srv_name + "' service"
                print(msg)
        except:
            msg = "Service '" + self.start_mission_srv_name + "' unavailable"
            rospy.logwarn(msg)

    def next_waypoint(self):
        try:
            trig_resp = self.next_waypoint_srv.call(TriggerRequest())
            if trig_resp.success:
                print(trig_resp.message)
            else:
                print(trig_resp.message)
                msg = "Failed to call '" + self.next_waypoint_srv_name + "' service"
                print(msg)
        except:
            msg = "Service '" + self.next_waypoint_srv_name + "' unavailable"
            rospy.logwarn(msg)

    def stop_mission(self):
        try:
            trig_resp = self.stop_mission_srv.call(TriggerRequest())
            if trig_resp.success:
                print(trig_resp.message)
            else:
                print(trig_resp.message)
                msg = "Failed to call '" + self.stop_mission_srv_name + "' service"
                print(msg)
        except:
            msg = "Service '" + self.stop_mission_srv_name + "' unavailable"
            rospy.logwarn(msg)

    def emergency_stop(self):
        try:
            trig_resp = self.emergency_stop_srv.call(TriggerRequest())
            if trig_resp.success:
                print(trig_resp.message)
            else:
                print(trig_resp.message)
                msg = "Failed to call '" + self.emergency_stop_srv_name + "' service"
                print(msg)
        except:
            msg = "Service '" + self.emergency_stop_srv_name + "' unavailable"
            rospy.logwarn(msg)

    def emergency_stop_resume(self):
        try:
            trig_resp = self.emergency_stop_resume_srv.call(TriggerRequest())
            if trig_resp.success:
                print(trig_resp.message)
            else:
                print(trig_resp.message)
                msg = "Failed to call '" + self.emergency_stop_resume_srv_name + "' service"
                print(msg)
        except:
            msg = "Service '" + self.emergency_stop_resume_srv_name + "' unavailable"
            rospy.logwarn(msg)

    def set_active_mode(self):
        mode = Mode()
        mode.mode = mode.ACTIVE

        try:
            trig_resp = self.set_active_mode_srv.call(mode)
            if trig_resp:
                print(trig_resp)
            else:
                msg = "Failed to call '" + self.set_active_mode_srv_name + "' service"
                print(msg)
        except:
            msg = "Service '" + self.set_active_mode_srv_name + "' unavailable"
            rospy.logwarn(msg)

    def status_label_update(self, msg):
        self.status_label.setText(msg.data)