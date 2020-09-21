#!/usr/bin/env python
import os
import rospy
import rospkg
import tf
from rr_viz.msg import TaskWaypoint
from PyQt5 import QtGui, QtWidgets, uic, QtCore
from PyQt5.QtWidgets import QApplication, QWidget, QInputDialog, QLineEdit, QFileDialog, QMessageBox, QListWidgetItem
from rr_viz.srv import BuildBT, BuildBTRequest
from std_msgs.msg import String
import RRTaskWaypoint


class QWaypointWidget(QWidget, RRTaskWaypoint):
    '''This is waypoint inherits logic from interactive_waypoints.waypoint and extends QWidget in order to implement GUI display on a QT list '''

    def __init__(self, parentList, msg=None):
        # Direct constructor calls.
        QWidget.__init__(self, parentList)
        RRTaskWaypoint.__init__(self, msg)

        self.allQHBoxLayout = QtWidgets.QHBoxLayout()  # Holding box
        self.idQLabel = QtWidgets.QLabel()  # First elem holds order number
        self.allQHBoxLayout.addWidget(self.idQLabel, 0)

        # Second elem is box holding labels
        self.textQVBoxLayout = QtWidgets.QVBoxLayout()
        self.textPoseQLabel = QtWidgets.QLabel()  # pose info label
        self.textNameQLabel = QtWidgets.QLabel()  # name  label
        self.textQVBoxLayout.addWidget(self.textPoseQLabel)
        self.textQVBoxLayout.addWidget(self.textNameQLabel)
        self.allQHBoxLayout.addLayout(self.textQVBoxLayout, 1)

        # Third elem is box holding drop downs
        self.comboBoxQVBoxLayout = QtWidgets.QVBoxLayout()
        self.taskSrvQComboBox = QtWidgets.QComboBox()
        self.taskSrvQComboBox.addItems(["None", "/continue_mission"])
        self.taskSrvQComboBox.currentIndexChanged.connect(self.taskSrvChange)
        self.taskSubTreeQComboBox = QtWidgets.QComboBox()
        self.taskSubTreeQComboBox.addItems(
            ["None", "InflateCostmaps", "DeflateCostmaps"])
        self.taskSubTreeQComboBox.currentIndexChanged.connect(
            self.taskSubTreeChange)

        self.comboBoxQVBoxLayout.addWidget(self.taskSrvQComboBox)
        self.comboBoxQVBoxLayout.addWidget(self.taskSubTreeQComboBox)
        self.allQHBoxLayout.addLayout(self.comboBoxQVBoxLayout, 2)

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

    def taskSrvChange(self, i):
        self.taskSrv = self.taskSrvQComboBox.itemText(i)

    def taskSubTreeChange(self, i):
        self.taskSubTree = self.taskSubTreeQComboBox.itemText(i)

    # overriden methods:

    def set_taskSrv(self, txt):
        RRTaskWaypoint.set_taskSrv(self, txt)
        index = self.taskSrvQComboBox.findText(
            txt, QtCore.Qt.MatchFixedString)
        if index >= 0:
            self.taskSrvQComboBox.setCurrentIndex(index)
        else:
            rospy.logerr("Tried to set task srv to non existant field")

    def set_taskSubTree(self, txt):
        RRTaskWaypoint.set_taskSubTree(self, txt)
        index = self.taskSubTreeQComboBox.findText(
            txt, QtCore.Qt.MatchFixedString)
        if index >= 0:
            self.taskSubTreeQComboBox.setCurrentIndex(index)
        else:
            rospy.logerr("Tried to set task subtree to non existant field")

    def _update(self):
        ''' This is called by Waypoint side. When update from rviz is received '''
        RRTaskWaypoint._update(self)
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
        RRTaskWaypoint.set_text(self, txt)
        self._setIdText(txt)
