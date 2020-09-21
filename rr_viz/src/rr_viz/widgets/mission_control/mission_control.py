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
from interactive_waypoints.waypoint import Waypoint
from rviz_minimap import RVizMinimap
import managers.file_management as file_management
from rr_viz.msg import TaskWaypoint
from rospy_message_converter import message_converter
from rr_viz.srv import BuildBT, BuildBTRequest
from std_msgs.msg import String
from ros_msgdict import msgdict
from helpers import rr_qt_helper

current_dir = os.path.dirname(os.path.abspath(__file__))
rr_viz_dir = _dir = os.path.dirname(os.path.dirname(current_dir))
Form, Base = uic.loadUiType(os.path.join(current_dir, "mission_control.ui"))


class MissionControlView(Base, Form):
    spawn_waypoint_signal = QtCore.pyqtSignal(PoseWithCovarianceStamped)
    set_mission_send_enabled = QtCore.pyqtSignal(bool)
    # pyqtSignals can only be created in  a class inheriting from QT.  Ideally InteractivePathMng should be a custom listWidget, but alas.
    # So these signals are created here instead and passed in. These signals are necessary because operations on the widget are invoked when  right click dropdown menu is  used(which is on a different thread).

    def __init__(self, parent=None):
        super(self.__class__, self).__init__(parent)
        self.setupUi(self)

        # Assuming that there is an RVizMinimapView as a sibling of this. #TODO later make this smarter
        try:
            self.rviz_minimap_manager = self.parent().findChildren(RVizMinimap)[
                0].rviz_manager
        except:
            rospy.logerr(
                "PathMaker Failed to find rviz-minimap, assuming 'map' as fixed frame")
            self.rviz_minimap_manager = None

        # Setup all button connections
        self.goToButton.clicked.connect(self.goto_slot)
        self.goToAllButton.clicked.connect(self.waypointList.gotoall)
        self.addButton.clicked.connect(self.add_slot)
        self.duplicateButton.clicked.connect(self.duplicate_slot)
        self.deleteAllButton.clicked.connect(self.delete_all_slot)
        self.deleteButton.clicked.connect(self.delete_this_slot)
        self.saveButton.clicked.connect(self.save_slot)
        self.loadButton.clicked.connect(self.load_slot)
        self.setup_mission_send_button()

        # Need to register a desctructor cuz QT object might live less than python.
        # https://machinekoder.com/how-to-not-shoot-yourself-in-the-foot-using-python-qt/
        self.destroyed.connect(self._unregister)

    def setup_spawn(self):
        # Setup subscriber  for pose spawning
        self.sub = rospy.Subscriber(
            "spawn_waypoint", PoseWithCovarianceStamped, lambda msg: self.spawn_waypoint_signal.emit(msg))
        self.spawn_waypoint_signal.connect(self.spawn_waypoint_slot)
        rospy.loginfo("waiting for build_bt to start")

    def setup_mission_send_button(self):
        def is_bt_up(self):
            try:
                rospy.wait_for_service("build_bt", rospy.Duration(3))
                return True
            except:
                # Exit if theres no service
                return False

        self.sendMissionButton.setEnabled(False)
        self.build_bt_srv = rospy.ServiceProxy("build_bt", BuildBT)
        self.sendMissionButton.clicked.connect(self.sendMissionButton_slot)

        # Setup state checker:
        self.set_mission_send_enabled.connect(
            self.sendMissionButton.setEnabled)
        self.sendMissionButton_state_checker = rr_qt_helper.StateCheckerTimer(
            is_bt_up,  self.set_mission_send_enabled, Hz=1./3.)
        self.state_checker.start()

    def spawn_waypoint_slot(self, msg):
        wp = QWaypointWidget()
        wp.set_pose(msg.pose)
        self.waypointList.append(wp)

    def _unregister(self):
        self.sub.unregister()
        del(self.waypointList)

    def goto_slot(self):
        if self.waypointList.get_selected_wp():
            self.waypointList.goto(self.waypointList.get_selected_wp())

    def add_slot(self):
        pose = PoseWithCovarianceStamped()
        pose.pose.pose.orientation.w = 1.
        if self.rviz_minimap_manager:
            pose.header.frame_id = str(
                self.rviz_minimap_manager.getFixedFrame())
        else:
            pose.header.frame_id = 'map'
        self.spawn_waypoint_signal.emit(pose)

    def duplicate_slot(self):
        if self.waypointList.get_selected_wp():
            self.waypointList.duplicate(self.waypointList.get_selected_wp())

    def delete_all_slot(self):
        for _ in range(self.waypointList.len()):
            self.waypointList.remove(
                self.waypointList.get_wp(self.waypointList.len()-1))

    def delete_this_slot(self):
        if self.waypointList.get_selected_wp():
            self.waypointList.remove(self.waypointList.get_selected_wp())

    def load_slot(self):
        filename = QFileDialog.getOpenFileName(
            self, 'Load Path', file_management.get_user_dir()+"/paths", "Config files (*.yaml)")[0]
        if filename:
            self.waypointList.loadFromPath(filename)

    def save_slot(self):
        filename = QFileDialog.getSaveFileName(
            self, 'Save Path', file_management.get_user_dir()+"/paths", "Config files (*.yaml)")[0]
        if filename:
            rospy.loginfo(filename[-5:])
            if filename[-5:] != ".yaml":
                filename = filename + ".yaml"
            self.waypointList.saveToPath(filename)

    def sendMissionButton_slot(self):
        btbr = BuildBTRequest()
        text, ok = QInputDialog.getText(
            self, 'Send Mission Dialog', 'Enter mission name')
        if ok:
            btbr.name = String(str(text))
            tasklist = []
            for ii in range(self.waypointList.len()):
                tasklist.append(self.waypointList.get_wp(ii).toMsg())

            btbr.tasklist = tasklist
            resp = self.build_bt_srv(btbr)
            if not resp.success.data:
                rospy.logerr("bt building failed")
