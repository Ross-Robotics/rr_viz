#!/usr/bin/env python
from PyQt5.QtWidgets import QVBoxLayout, QHBoxLayout, QWidget, QLabel, QPushButton, QGridLayout, QInputDialog
from PyQt5.QtGui import QFont
from PyQt5.QtCore import Qt
from PyQt5 import QtCore

import actionlib
import rospy
from std_srvs.srv import Trigger, TriggerRequest
from rr_custom_msgs.msg import RecordPathAction, RecordPathGoal
from rr_custom_msgs.msg import TrackPathAction, TrackPathGoal
from helpers import rr_qt_helper
import string

class PathNavigationTools(QWidget):
    set_enable_record = QtCore.pyqtSignal(bool)
    set_enable_follow = QtCore.pyqtSignal(bool)

    def __init__(self, parent):
        super(QWidget, self).__init__(parent)

        # Get parameters
        self.file_path = rospy.get_param("/path_navigation/file_path", "")
                
        # Setup services
        self.stop_recording_srv_name = "/path_recorder/finish_path"
        self.stop_recording_srv = rospy.ServiceProxy(self.stop_recording_srv_name, Trigger)

        self.get_path_files_srv_name = "/path_tracker/get_path_files"
        self.get_path_files_srv = rospy.ServiceProxy(self.get_path_files_srv_name, Trigger)

        # Setup action clients
        self.start_recording_action_name = "/path_recorder/record"
        self.tracking_action_name = "path_tracker/start_tracking"
        self.start_recording_action = actionlib.SimpleActionClient(self.start_recording_action_name, RecordPathAction)
        self.tracking_action = actionlib.SimpleActionClient(self.tracking_action_name, TrackPathAction)

        self.v_layout = QVBoxLayout()

        # Title
        self.title_label = QLabel('Path Navigation Tools')
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

        self.enable_recording_buttons(False)
        self.set_enable_record.connect(self.enable_recording_buttons)

        self.follow_saved_path_button = QPushButton('Follow Saved Path')
        self.follow_saved_path_button.pressed.connect(self.start_following)

        self.stop_following_button = QPushButton('Stop Following')
        self.stop_following_button.pressed.connect(self.stop_following)

        self.enable_following_buttons(False)
        self.set_enable_follow.connect(self.enable_following_buttons)

        self.grid_layout.addWidget(self.start_recording_button, 0, 0)
        self.grid_layout.addWidget(self.stop_recording_button, 0, 1)
        self.grid_layout.addWidget(self.follow_saved_path_button, 1, 0)
        self.grid_layout.addWidget(self.stop_following_button, 1, 1)

        self.v_layout.addLayout(self.grid_layout)

        self.setLayout(self.v_layout)
        
        self.record_state_checker = rr_qt_helper.StateCheckerTimer(
            self.is_record_up, self.set_enable_record, Hz=1./3.)
        self.record_state_checker.start()

        self.follow_state_checker = rr_qt_helper.StateCheckerTimer(
            self.is_record_up, self.set_enable_follow, Hz=1./3.)
        self.follow_state_checker.start()

    def start_recording(self):
        path_name, ok = QInputDialog.getText(self, "Path file name", "Specify file name to save recorded path:")

        if ok:
            if path_name != "":
                self.file_path = string.replace(self.file_path, "path.txt", path_name + ".txt")
            
            self.start_recording_action.send_goal(RecordPathGoal(file_path=self.file_path))

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
        file_path_to_search = string.replace(self.file_path, "/path.txt", "")

        try:
            trig_resp = self.get_path_files_srv.call(TriggerRequest())
        except Exception as e:
            rospy.logwarn_throttle(10, "Failed to fetch path files: {}".format(e))
            return

        if trig_resp.success:
            path_files = str(trig_resp.message).split(",")

            path_to_follow, ok = QInputDialog.getItem(self, "Select path to follow", "Available paths:", path_files, 0, False)
            path = file_path_to_search + "/" + path_to_follow
            print(path)
            if ok:
                self.tracking_action.send_goal(TrackPathGoal(file_path=path))
        else:
            rospy.logwarn_throttle(10, "Failed to get path files")

    def stop_following(self):
        self.tracking_action.cancel_all_goals()

    def enable_recording_buttons(self, enabled):
        self.start_recording_button.setEnabled(enabled)
        self.stop_recording_button.setEnabled(enabled)

    def enable_following_buttons(self, enabled):
        self.follow_saved_path_button.setEnabled(enabled)
        self.stop_following_button.setEnabled(enabled)

    def is_record_up(self):
        return self.start_recording_action.wait_for_server(timeout=rospy.Duration(2.))

    def is_follow_up(self):
        return self.start_recording_action.wait_for_server(timeout=rospy.Duration(2.))
