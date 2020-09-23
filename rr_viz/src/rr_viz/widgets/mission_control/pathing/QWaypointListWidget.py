#!/usr/bin/env python
import os
import rospy
import rospkg
from PyQt5 import QtGui, QtWidgets, uic, QtCore
from PyQt5.QtWidgets import QApplication, QWidget, QInputDialog, QLineEdit, QFileDialog, QMessageBox, QListWidgetItem, QListWidget
from rr_viz.srv import BuildBT, BuildBTRequest
from std_msgs.msg import String
from QWaypointWidget import QWaypointWidget
from RRTaskWaypointList import RRTaskWaypointList
from interactive_waypoints.waypoint_actions import WaypointMoveBaseAction
from ros_msgdict import msgdict


class QWaypointListWidget(RRTaskWaypointList, QListWidget):
    ''' This class abstracts and adapts the RRTaskWaypointList to be suitable for  use with QT.
    Main note is: overridden methos splits logic into business and GUI. business is implemented via calls to super. gui via emiting to slots.'''
    duplicate_signal = QtCore.pyqtSignal(QWaypointWidget)
    del_signal = QtCore.pyqtSignal(QWaypointWidget)
    ins_signal = QtCore.pyqtSignal(int, QWaypointWidget)
    chgid_signal = QtCore.pyqtSignal(QWaypointWidget, int)
    waypoint_class = QWaypointWidget

    def __init__(self, parent=None):
        RRTaskWaypointList.__init__(self)
        QListWidget.__init__(self)
        self.setSortingEnabled(False)
        # Adding movebase action by default.it is always intended
        self.mb_action = WaypointMoveBaseAction(
            rospy.get_param("~move_base_namespace", "/move_base"))
        # Disabling these methods cuz inconsistent GUI
        self._menu_handler.reApply(self._server)
        self._server.applyChanges()
        # general connections
        self.currentItemChanged.connect(self._list_selected)
        # drop menu connections
        self.del_signal.connect(
            lambda wp: self.takeItem(self.row(wp.item)))
        self.duplicate_signal.connect(
            lambda wp: RRTaskWaypointList.duplicate(self, wp))
        self.ins_signal.connect(self.insert_slot)
        self.chgid_signal.connect(self.changeID_slot)
        self.attach_movebase_menu_actions()

    def attach_qbutton_action(self, qbutton, exec_cb, check_cb):
        qbutton.clicked.connect(exec_cb)
        setEnabledSignal = QtCore.pyqtSignal(bool)
        setEnabledSignal.connect(qbutton.setEnabled)

        self.action_state_timers.append(
            rospy.Timer(rospy.Duration(1.0), lambda _: qbutton.setEnabledSignal.emit(check_cb())))
        return setEnabledSignal

    def attach_movebase_menu_actions(self):
        # Add actions to context menu
        self.attach_menu_action(
            "goto", self.mb_action.goto_action, self.mb_action.is_connected)
        self.attach_menu_action(
            "gotoall", self.mb_action.gotoall_action, self.mb_action.is_connected)
        self.attach_menu_action(
            "cancel goto", self.mb_action.cancel_goals_action, self.mb_action.is_connected)

    def _list_selected(self):
        # Highlight logic:
        for ii in range(self.len()):
            self.get_wp(ii).set_highlight(False)
        if self.currentRow() >= 0:
            self.get_selected_wp().set_highlight(True)

    def get_selected_wp(self):
        if self.currentRow() >= 0:
            return self.itemWidget(self.item(self.currentRow()))
        else:
            rospy.logwarn("Requested selected wp, but no wp was selected")
            return None

    def new_waypoint(self, msg=None):
        return QWaypointWidget(self, msg)

    def pop(self, wp):
        self.del_signal.emit(wp)
        wp = RRTaskWaypointList.pop(self, wp)
        return wp

    def insert(self, idx, wp):
        RRTaskWaypointList.insert(self, idx, wp)  # Call to super
        self.ins_signal.emit(idx, wp)

    def insert_slot(self, idx, wp):
        self.insertItem(idx, wp.item)
        self.setItemWidget(wp.item, wp)

    def duplicate(self, wp):
        self.duplicate_signal.emit(wp)

    def changeID(self, wp_old, new_id):
        # Due to complexitie, changeID must act as pop insert operation now.
        if not isinstance(wp_old, QWaypointWidget):  # if id or name passed
            wp_old = self.get_wp(wp_old)
        self.chgid_signal.emit(wp_old, new_id)

    def changeID_slot(self, wp_old, new_id):
        # Due to complexitie, changeID must act as pop insert operation now.
        wp = self.pop(wp_old)
        self.insert(new_id, QWaypointWidget(self, wp.save_to_msg()))
