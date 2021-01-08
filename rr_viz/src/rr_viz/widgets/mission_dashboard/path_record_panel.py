#!/usr/bin/env python
import os
import rospy
from PyQt5 import QtGui, QtWidgets, uic, QtCore
from PyQt5.QtWidgets import QApplication, QWidget, QInputDialog, QLineEdit, QFileDialog, QMessageBox, QListWidgetItem
from rr_2d_navigation.msg import RecordPathAction, RecordPathResult, RecordPathFeedback, RecordPathActionGoal
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
        self.statusLabel.setup("/path_recorder/status")
        self._client = actionlib.SimpleActionClient('/path_recorder/record', RecordPathAction)
        self.state_checker = rr_qt_helper.StateCheckerTimer(
            self.is_record_up, self.set_enabled, Hz=1./3.)
        self.state_checker.start()        

    def is_record_up(self):        
        return self._client.wait_for_server(timeout=rospy.Duration(2.))    

    def startRecordingSlot(self):
        self._client.send_goal(RecordPathActionGoal()) #Assuming declared via param

# Actions:
# path_recorder/record
# service finish_path
