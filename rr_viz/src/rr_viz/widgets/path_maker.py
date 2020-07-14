#!/usr/bin/env python

import os
import rospy
import rospkg
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped
from functools import partial
from PyQt5 import QtGui, QtWidgets, uic, QtCore
from PyQt5.QtWidgets import QApplication, QWidget, QInputDialog, QLineEdit, QFileDialog, QMessageBox, QListWidgetItem

from interactive_waypoints.waypoint_list import InteractiveWaypointList
from interactive_waypoints.waypoint import Waypoint
from rviz_minimap import RVizMinimap
import managers.file_management as file_management

current_dir = os.path.dirname(os.path.abspath(__file__))
rr_viz_dir = _dir = os.path.dirname(os.path.dirname(current_dir))
Form, Base = uic.loadUiType(os.path.join(current_dir, "path_maker.ui"))


class QWaypointWidget(QWidget, Waypoint):
    '''This is waypoint inherits logic from interactive_waypoints.waypoint and extends QWidget in order to implement GUI display on a QT list '''

    def __init__(self, iw_srv, pose, parentList):
        # Direct constructor calls.
        QWidget.__init__(self, parentList)
        Waypoint.__init__(self, iw_srv, pose)

        # create custom Waypoint display
        self.textQVBoxLayout = QtWidgets.QVBoxLayout()
        self.textPoseQLabel = QtWidgets.QLabel()
        self.textNameQLabel = QtWidgets.QLabel()
        self.textQVBoxLayout.addWidget(self.textPoseQLabel)
        self.textQVBoxLayout.addWidget(self.textNameQLabel)

        self.allQHBoxLayout = QtWidgets.QHBoxLayout()
        self.idQLabel = QtWidgets.QLabel()

        self.allQHBoxLayout.addWidget(self.idQLabel, 0)
        self.allQHBoxLayout.addLayout(self.textQVBoxLayout, 1)

        self.setLayout(self.allQHBoxLayout)

        # set styles
        self.textPoseQLabel.setStyleSheet(
            ''' color: rgb(0, 0, 0);''')
        self.textNameQLabel.setStyleSheet(
            ''' font-size: 8pt; color: rgb(146, 150, 147);''')
        self.idQLabel.setStyleSheet(
            ''' font-size: 12pt; color: rgb(100, 100, 100);''')
        # Widget keeps track of item it is assigned to
        # Must call this to make sure the size is ok
        self.item = QListWidgetItem(None)
        self.item.setSizeHint(self.sizeHint())
        self._update()

    def _setPoseText(self, text):
        self.textPoseQLabel.setText(text)

    def _setNameText(self, text):
        self.textNameQLabel.setText(text)

    def _setIdText(self, text):
        self.idQLabel.setText(text)

    # overriden methods:
    def _update(self):
        ''' This is called by Waypoint side. When update from rviz is received '''
        Waypoint._update(self)
        pose = self._int_marker.pose
        q = (pose.orientation.x,
             pose.orientation.y,
             pose.orientation.z,
             pose.orientation.w)
        eul = tf.transformations.euler_from_quaternion(q)
        self._setPoseText("x: {:+.2f}   y: {:+.2f}   th: {:+.2f}".format(
            pose.position.x, pose.position.y, eul[2]))
        self._setNameText(self._name)

    def set_text(self, txt):
        Waypoint.set_text(self, txt)
        self._setIdText(txt)


class InteractivePathMng(InteractiveWaypointList):
    ''' This class abstracts and adapts the InteractiveWaypointList to be suitable for  use with QT. 
    Main note is: overridden methos splits logic into business and GUI. business is implemented via calls to super. gui via emiting to slots.'''

    def __init__(self, srv_name, qlistwidget, dup_signal, del_signal, ins_signal, chgid_signal):
        InteractiveWaypointList.__init__(self, srv_name, mb=True, fm=True)
        self.qlistwidget = qlistwidget
        self.qlistwidget.setSortingEnabled(False)
        # Disabling these methods cuz inconsistent GUI
        self._menu_handler.reApply(self._server)
        self._server.applyChanges()
        # general connegtions
        self.qlistwidget.currentItemChanged.connect(self._list_selected)
        # drop menu connections
        self.del_signal = del_signal
        self.del_signal.connect(
            lambda wp: self.qlistwidget.takeItem(self.qlistwidget.row(wp.item)))
        self.duplicate_signal = dup_signal
        self.duplicate_signal.connect(
            lambda wp: InteractiveWaypointList.duplicate(self, wp))
        self.ins_signal = ins_signal
        self.ins_signal.connect(self.insert_slot)
        self.chgid_signal = chgid_signal
        self.chgid_signal.connect(self.changeID_slot)

    def _list_selected(self):
        # Highlight logic:
        for ii in range(self.len()):
            self.get_wp(ii).set_highlight(False)
        if self.qlistwidget.currentRow() >= 0:
            # rospy.loginfo("selected item {}  wp {}".format(self.qlistwidget.item(
            #     self.qlistwidget.currentRow()), self.get_selected_wp()))
            self.get_selected_wp().set_highlight(True)

    def get_selected_wp(self):
        if self.qlistwidget.currentRow() >= 0:
            return self.qlistwidget.itemWidget(self.qlistwidget.item(self.qlistwidget.currentRow()))
        else:
            rospy.logwarn("Requested selected wp, but no wp was selected")
            return None

    # Overridden methods
    def get_new_wp(self, pose):
        return QWaypointWidget(self._server, pose, self.qlistwidget)

    def pop(self, wp):
        self.del_signal.emit(wp)
        wp = InteractiveWaypointList.pop(self, wp)
        return wp

    def insert(self, idx, wp):
        InteractiveWaypointList.insert(self, idx, wp)  # Call to super
        self.ins_signal.emit(idx, wp)

    def insert_slot(self, idx, wp):
        self.qlistwidget.insertItem(idx, wp.item)
        self.qlistwidget.setItemWidget(wp.item, wp)

    def list_items(self):
        # This is for debugging
        rospy.loginfo("listing items")
        for ii in range(self.qlistwidget.count()):
            rospy.loginfo(self.qlistwidget.item(ii))
        rospy.loginfo("end of list")

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
        self.insert(new_id, self.get_new_wp(wp.get_pose()))


class PathMakerWidget(Base, Form):
    spawn_waypoint_signal = QtCore.pyqtSignal(PoseWithCovarianceStamped)
    # pyqtSignals can only be created in  a class inheriting from QT.  Ideally InteractivePathMng should be a custom listWidget, but alas.
    # So these signals are created here instead and passed in. These signals are necessary because operations on the widget are invoked when  right click dropdown menu is  used(which is on a different thread).
    list_duplicate_signal = QtCore.pyqtSignal(QWaypointWidget)
    list_delete_signal = QtCore.pyqtSignal(QWaypointWidget)
    list_insert_signal = QtCore.pyqtSignal(int, QWaypointWidget)
    list_changeid_signal = QtCore.pyqtSignal(QWaypointWidget, int)

    def __init__(self, parent=None):
        super(self.__class__, self).__init__(parent)
        self.setupUi(self)
        # Initialise Waypoint List business logic
        self.int_waypoints = InteractivePathMng(
            "path_maker_srv", self.waypointList, self.list_duplicate_signal, self.list_delete_signal, self.list_insert_signal, self.list_changeid_signal)
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
        self.goToAllButton.clicked.connect(self.int_waypoints.gotoall)
        self.addButton.clicked.connect(self.add_slot)
        self.duplicateButton.clicked.connect(self.duplicate_slot)
        self.deleteAllButton.clicked.connect(self.delete_all_slot)
        self.deleteButton.clicked.connect(self.delete_this_slot)
        self.saveButton.clicked.connect(self.save_slot)
        self.loadButton.clicked.connect(self.load_slot)

        # Setup subscriber  for pose spawning
        self.sub = rospy.Subscriber(
            "spawn_waypoint", PoseWithCovarianceStamped, lambda msg: self.spawn_waypoint_signal.emit(msg))
        self.spawn_waypoint_signal.connect(self.spawn_waypoint_slot)

        # Need to register a desctructor cuz QT object might live less than python.
        # https://machinekoder.com/how-to-not-shoot-yourself-in-the-foot-using-python-qt/
        self.destroyed.connect(self._unregister)

    def spawn_waypoint_slot(self, msg):
        self.int_waypoints.append(self.int_waypoints.get_new_wp(msg))

    def _unregister(self):
        self.sub.unregister()
        del(self.int_waypoints)

    def goto_slot(self):
        if self.int_waypoints.get_selected_wp():
            self.int_waypoints.goto(self.int_waypoints.get_selected_wp())

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
        if self.int_waypoints.get_selected_wp():
            self.int_waypoints.duplicate(self.int_waypoints.get_selected_wp())

    def delete_all_slot(self):
        for _ in range(self.int_waypoints.len()):
            self.int_waypoints.remove(
                self.int_waypoints.get_wp(self.int_waypoints.len()-1))

    def delete_this_slot(self):
        if self.int_waypoints.get_selected_wp():
            self.int_waypoints.remove(self.int_waypoints.get_selected_wp())

    def load_slot(self):
        filename = QFileDialog.getOpenFileName(
            self, 'Load Path', file_management.get_user_dir()+"/paths", "Config files (*.yaml)")[0]
        if filename:
            self.int_waypoints.loadFromPath(filename)

    def save_slot(self):
        filename = QFileDialog.getSaveFileName(
            self, 'Save Path', file_management.get_user_dir()+"/paths", "Config files (*.yaml)")[0]
        if filename:
            rospy.loginfo(filename[-5:])
            if filename[-5:] != ".yaml":
                filename = filename + ".yaml"
            self.int_waypoints.saveToPath(filename)
