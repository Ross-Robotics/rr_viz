#!/usr/bin/env python

import os
import rospy
import rospkg
import rosparam
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from PyQt5 import QtGui, QtWidgets, uic, QtCore
from PyQt5.QtWidgets import QApplication, QWidget, QFileDialog
import managers.file_management as file_management
from pathing.QWaypointWidget import QWaypointWidget
from rr_viz.msg import TaskWaypoint as TaskWaypointMsg

current_dir = os.path.dirname(os.path.abspath(__file__))
rr_viz_dir = _dir = os.path.dirname(os.path.dirname(current_dir))
Form, Base = uic.loadUiType(os.path.join(current_dir, "mission_editor.ui"))


class MissionEditorWidget(Base, Form):
    spawn_waypoint_signal = QtCore.pyqtSignal(PoseWithCovarianceStamped)
    set_mission_send_enabled = QtCore.pyqtSignal(bool)

    def __init__(self, parent=None):
        super(self.__class__, self).__init__(parent)
        self.setupUi(self)

        # Setup all button connections
        self.addButton.clicked.connect(self.add_slot)  # TODO
        self.duplicateButton.clicked.connect(self.duplicate_slot)
        self.deleteAllButton.clicked.connect(self.delete_all_slot)
        self.deleteButton.clicked.connect(self.delete_this_slot)
        self.saveButton.clicked.connect(self.save_slot)
        self.loadButton.clicked.connect(self.load_slot)
        self.setup_spawn()
        # Need to register a desctructor cuz QT object might live less than python.
        # https://machinekoder.com/how-to-not-shoot-yourself-in-the-foot-using-python-qt/
        self.destroyed.connect(self._unregister)

    def setup_spawn(self):
        # Setup subscriber  for pose spawning
        self.sub = rospy.Subscriber(
            "spawn_waypoint", PoseWithCovarianceStamped, lambda msg: self.spawn_waypoint_signal.emit(msg))
        self.spawn_waypoint_signal.connect(self.spawn_waypoint_slot)
        rospy.loginfo("waiting for build_bt to start")

    def spawn_waypoint_slot(self, pwcs_msg):
        msg = TaskWaypointMsg()
        pose_stamped = PoseStamped()
        pose_stamped.header = pwcs_msg.header
        pose_stamped.pose = pwcs_msg.pose.pose
        msg.pose_stamped = pose_stamped
        self.waypointList.append(self.waypointList.new_waypoint(msg))

    def _unregister(self):
        self.sub.unregister()
        del(self.waypointList)

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
            self.waypointList.saveToPath(filename, "Mission" +
                                         str(rospy.Time.now()))
