#!/usr/bin/env python
import sys
from PyQt5.QtWidgets import QVBoxLayout, QHBoxLayout, QWidget, QLabel, QPushButton, QGridLayout
from PyQt5.QtGui import QFont
from PyQt5.QtCore import Qt

import actionlib
import rospy
from std_srvs.srv import Trigger, TriggerRequest
from rr_custom_msgs.msg import RecordPathAction, RecordPathResult, RecordPathFeedback, RecordPathGoal
from rr_custom_msgs.msg import TrackPathAction, TrackPathResult, TrackPathFeedback, TrackPathGoal

class PathRecording(QWidget):
    def __init__(self, parent):
        super(QWidget, self).__init__(parent)

        # Setup services
        self.stop_recording_srv_name = "/path_recorder/finish_path"
        self.stop_recording_srv = rospy.ServiceProxy(self.stop_recording_srv_name, Trigger)

        # Setup action clients
        self.start_recording_action_name = "/path_recorder/record"
        self.tracking_action_name = "path_tracker/start_tracking"
        self.start_recording_action = actionlib.SimpleActionClient(self.start_recording_action_name, RecordPathAction)
        self.tracking_action = actionlib.SimpleActionClient(self.tracking_action_name, TrackPathAction)

        self.v_layout = QVBoxLayout()

        # Title
        self.title_label = QLabel('Path Recording')
        self.title_label.setFont(QFont('Ubuntu', 11, QFont.Bold))
        self.title_label.setAlignment(Qt.AlignRight)
        self.v_layout.addWidget(self.title_label)

        # Status
        self.h_layout = QHBoxLayout()
        self.status_text_label = QLabel('Status:')
        self.status_text_label.setFont(QFont('Ubuntu', 10, QFont.Bold))
        self.status_label = QLabel('')
        self.h_layout.addWidget(self.status_text_label, 3)
        self.h_layout.addWidget(self.status_label, 7)
        
        self.v_layout.addLayout(self.h_layout)

        # Buttons
        self.grid_layout = QGridLayout()

        self.start_recording_button = QPushButton('Start Recording')
        self.start_recording_button.pressed.connect(self.start_recording)

        self.stop_recording_button = QPushButton('Stop Recording')
        self.stop_recording_button.pressed.connect(self.stop_recording)

        self.follow_saved_path_button = QPushButton('Follow Saved Path')
        self.follow_saved_path_button.pressed.connect(self.start_following)

        self.stop_following_button = QPushButton('STOP FOLLOWING')
        self.stop_following_button.pressed.connect(self.stop_following)
        self.grid_layout.addWidget(self.start_recording_button, 0, 0)
        self.grid_layout.addWidget(self.stop_recording_button, 0, 1)
        self.grid_layout.addWidget(self.follow_saved_path_button, 1, 0)
        self.grid_layout.addWidget(self.stop_following_button, 1, 1)

        self.v_layout.addLayout(self.grid_layout)

        self.setLayout(self.v_layout)

    def start_recording(self):
        self.start_recording_action.send_goal(RecordPathGoal(file_path=""))

    def stop_recording(self):
        try:
            trig_resp = self.stop_recording_srv.call(TriggerRequest())
            if trig_resp.success:
                print(trig_resp.message)
            else:
                print(trig_resp.message)
                msg = "Failed to call '" + self.stop_mission_srv_name + "' service"
                print(msg)
        except:
            msg = "Service '" + self.stop_mission_srv_name + "' unavailable"
            rospy.logwarn(msg)

    def start_following(self):
        self.tracking_action.send_goal(TrackPathGoal(file_path=""))

    def stop_following(self):
        self.tracking_action.cancel_all_goals()