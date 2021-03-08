#!/usr/bin/env python
import os
import rospy
from PyQt5 import QtGui, QtWidgets, uic, QtCore
from PyQt5.QtWidgets import QApplication, QWidget, QInputDialog, QLineEdit, QFileDialog, QMessageBox, QListWidgetItem
from std_srvs.srv import Trigger
from rr_custom_msgs.srv import SetMode, SetModeRequest, SetModeResponse
from rr_custom_msgs.msg import Mode

current_dir = os.path.dirname(os.path.abspath(__file__))
rr_viz_dir = _dir = os.path.dirname(os.path.dirname(current_dir))
Form, Base = uic.loadUiType(os.path.join(
    current_dir, "mission_control_panel.ui"))


class MissionControlPanelWidget(Base, Form):
    ''' The only reason this exists so that we can connect mission_editor to command_panel  easily '''

    def __init__(self, parent=None):
        super(self.__class__, self).__init__(parent)
        self.setupUi(self)

        self.startMissionButton = self.findChildren(QtWidgets.QPushButton, "startMissionButton")[0]
        self.stopMissionButton = self.findChildren(QtWidgets.QPushButton, "stopMissionButton")[0]
        self.eStopButton = self.findChildren(QtWidgets.QPushButton, "eStopButton")[0]
        self.eResumeButton = self.findChildren(QtWidgets.QPushButton, "eResumeButton")[0]
        self.setActiveButton = self.findChildren(QtWidgets.QPushButton, "setActiveButton")[0]
        self.nextWaypointButton = self.findChildren(QtWidgets.QPushButton, "nextWaypointButton")[0]

        self.missionStatusLabel = self.findChildren(QtWidgets.QLabel, "missionStatusLabel")[0]

        self.setActiveButton.setup(
            "/robot_interface/set_robot_mode",SetMode, self.setModeActive)
        self.nextWaypointButton.setup(
            "/robot_interface/continue_mission", Trigger)
        self.startMissionButton.setup(
            "/robot_interface/start_mission", Trigger)
        self.stopMissionButton.setup(
            "/robot_interface/stop_mission", Trigger)
        self.eStopButton.setup("/robot_interface/emergency_stop", Trigger)
        self.eResumeButton.setup(
            "/robot_interface/cancel_emergency_stop", Trigger)
        self.missionStatusLabel.setup("/robot_interface/mission_status")

    def setModeActive(self):
        mode = Mode()
        mode.mode = mode.ACTIVE
        return mode

    def setModeInactive(self):
        mode = Mode()
        mode.mode = mode.INACTIVE
        return mode
