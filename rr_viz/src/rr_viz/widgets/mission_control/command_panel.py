#!/usr/bin/env python

import os
import rospy
import rospkg
import rosparam
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped
from functools import partial
from PyQt5 import QtGui, QtWidgets, uic, QtCore
from PyQt5.QtWidgets import QApplication, QWidget, QInputDialog, QLineEdit, QFileDialog, QMessageBox, QListWidgetItem

from interactive_waypoints.waypoint_list import InteractiveWaypointList
from interactive_waypoints.waypoint_actions import WaypointMoveBaseAction, WaypointSaveLoadAction
import managers.file_management as file_management
from rospy_message_converter import message_converter
from rr_viz.srv import BuildBT, BuildBTRequest
from mission_actions import BuildBTAction
from helpers import rr_qt_helper

current_dir = os.path.dirname(os.path.abspath(__file__))
rr_viz_dir = _dir = os.path.dirname(os.path.dirname(current_dir))
Form, Base = uic.loadUiType(os.path.join(current_dir, "command_panel.ui"))


class MissionCommandPanelWidget(Base, Form):
    set_goto_enabled = QtCore.pyqtSignal(bool)
    set_mission_send_enabled = QtCore.pyqtSignal(bool)

    def __init__(self, parent=None):
        super(self.__class__, self).__init__(parent)
        self.setupUi(self)
        self.sendMissionButton.setEnabled(False)
        self.setGotoButtonsEnabled(False)
        self.set_goto_enabled.connect(self.setGotoButtonsEnabled)
        self.set_mission_send_enabled.connect(
            self.sendMissionButton.setEnabled)
        self.btAction = BuildBTAction()

        # Setup state checker:
        self.bt_state_checker = rr_qt_helper.StateCheckerTimer(
            self.btAction.is_connected,  self.set_mission_send_enabled, Hz=1./3.)
        self.bt_state_checker.start()

    def connectToWaypointList(self, waypointList):
        self.sendMissionButton.clicked.connect(
            lambda: self.btAction.build_bt_action(waypointList))

        self.mbAction = waypointList.mb_action
        self.mb_state_checker = rr_qt_helper.StateCheckerTimer(
            self.mbAction.is_connected,  self.set_goto_enabled, Hz=1./3.)
        self.mb_state_checker.start()

        self.goToButton.clicked.connect(waypointList.goto_selected_slot)
        self.goToAllButton.clicked.connect(waypointList.goto_all_slot)

    def setGotoButtonsEnabled(self, enabled):
        self.goToButton.setEnabled(enabled)
        self.goToAllButton.setEnabled(enabled)
