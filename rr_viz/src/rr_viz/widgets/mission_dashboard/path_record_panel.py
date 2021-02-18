#!/usr/bin/env python
import os
import rospy
from PyQt5 import QtGui, QtWidgets, uic, QtCore
from PyQt5.QtWidgets import QApplication, QWidget, QInputDialog, QLineEdit, QFileDialog, QMessageBox, QListWidgetItem
from rr_custom_msgs.msg import RecordPathAction, RecordPathResult, RecordPathFeedback, RecordPathGoal
from rr_custom_msgs.msg import TrackPathAction, TrackPathResult, TrackPathFeedback, TrackPathGoal
from std_srvs.srv import Trigger
from helpers import rr_qt_helper
import actionlib

current_dir = os.path.dirname(os.path.abspath(__file__))
rr_viz_dir = _dir = os.path.dirname(os.path.dirname(current_dir))
Form, Base = uic.loadUiType(os.path.join(current_dir, "path_record_panel.ui"))


class PathRecordPanelWidget(Base, Form):
    ''' The only reason this exists so that we can connect mission_editor to command_panel  easily '''
    set_enabled = QtCore.pyqtSignal(bool)
    def __init__(self, parent=None):
        super(self.__class__, self).__init__(parent)
        self.setupUi(self)
        self.set_enabled.connect(self.setEnabled)
        self.stopRecordingButton.setup(
            "/path_recorder/finish_path", Trigger)
        self.startRecordingButton.clicked.connect(self.startRecordingSlot)
        self.followSavedPathButton.clicked.connect(self.followSavedPath)
        self.stopFollowingPathButton.clicked.connect(self.stopFollowingPath)
        self.statusLabel.setup("/path_recorder/status")
        self._recorder_client = actionlib.SimpleActionClient('/path_recorder/record', RecordPathAction)
        self._tracker_client = actionlib.SimpleActionClient('/path_tracker/start_tracking', TrackPathAction)
        self.state_checker = rr_qt_helper.StateCheckerTimer(
            self.is_record_up, self.set_enabled, Hz=1./3.)
        self.state_checker.start()

    def is_record_up(self):
        return self._recorder_client.wait_for_server(timeout=rospy.Duration(2.))

    def startRecordingSlot(self):
        self._recorder_client.send_goal(RecordPathGoal(file_path="")) #Assuming declared via param

    def followSavedPath(self):
        self._tracker_client.send_goal(TrackPathGoal(file_path=""))

    def stopFollowingPath(self):
        self._tracker_client.cancel_all_goals()


# Actions:
# path_recorder/record
# service finish_path
