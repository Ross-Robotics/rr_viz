#!/usr/bin/env python
import os
import rospy
import rospkg
from rr_viz.msg import TaskWaypoint
from PyQt5 import QtGui, QtWidgets, uic, QtCore
from PyQt5.QtWidgets import QApplication, QWidget, QInputDialog, QLineEdit, QFileDialog, QMessageBox, QListWidgetItem, QListWidget
from rr_viz.srv import BuildBT, BuildBTRequest
from std_msgs.msg import String
import QWaypointWidget
import RRTaskWaypointList
from ros_msgdict import msgdict


class QWaypointListWidget(RRTaskWaypointList, QListWidget):
    ''' This class abstracts and adapts the RRTaskWaypointList to be suitable for  use with QT.
    Main note is: overridden methos splits logic into business and GUI. business is implemented via calls to super. gui via emiting to slots.'''
    list_duplicate_signal = QtCore.pyqtSignal(QWaypointWidget)
    list_delete_signal = QtCore.pyqtSignal(QWaypointWidget)
    list_insert_signal = QtCore.pyqtSignal(int, QWaypointWidget)
    list_changeid_signal = QtCore.pyqtSignal(QWaypointWidget, int)

    def __init__(self):
        RRTaskWaypointList.__init__(self)
        QListWidget.__init__(self)
        self.setSortingEnabled(False)

        # Disabling these methods cuz inconsistent GUI
        self._menu_handler.reApply(self._server)
        self._server.applyChanges()
        # general connegtions
        self.currentItemChanged.connect(self._list_selected)
        # drop menu connections
        self.del_signal.connect(
            lambda wp: self.takeItem(self.row(wp.item)))
        self.duplicate_signal.connect(
            lambda wp: RRTaskWaypointList.duplicate(self, wp))
        self.ins_signal.connect(self.insert_slot)
        self.chgid_signal.connect(self.changeID_slot)

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
        # This is a workaround:
        # duplicate creats a new waypoint item. This must be  done from QT thread. hence this is all wrapped
        self.duplicate_signal.emit(wp)

    def changeID(self, wp_old, new_id):
        # Due to complexitie, changeID must act as pop insert operation now.
        if not isinstance(wp_old, Waypoint):  # if id or name passed
            wp_old = self.get_wp(wp_old)
        self.chgid_signal.emit(wp_old, new_id)

    def changeID_slot(self, wp_old, new_id):
        # Due to complexitie, changeID must act as pop insert operation now.
        wp = self.pop(wp_old)
        self.insert(new_id, QWaypointWidget(self, wp.save_to_msg()))
