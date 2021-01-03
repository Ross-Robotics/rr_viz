import os
from PyQt5 import QtCore, QtGui, QtWidgets, uic
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import pyqtSlot
import rospy
from rviz_minimap import RVizMinimap
from rviz_dev import RVizDev
from rviz_sphere import RVizSphere
import rviz


class ROSTopicLabel(QtWidgets.QLabel):
    def __init__(self, parent=None):
        super(self.__class__, self).__init__(parent)
        self.currentChanged.connect(self.currentChangedSlot)
        self.rviz_ref = dict()
        self.isSetup = False

    def setup_rviz_ref(self):
        # HARDCODE for test
        self.rviz_ref[0] = self.findChildren(RVizDev)
        self.rviz_ref[1] = self.findChildren(RVizSphere)
        self.rviz_ref[2] = self.findChildren(RVizMinimap)
        self.isSetup = all([len(ref) == 0 for ref in self.rviz_ref.values()])

    def currentChangedSlot(self, curr_index):
        if not self.isSetup:
            self.setup_rviz_ref()
            return
        for index in range(self.count()):
            self.widget(index).setEnabled(index == curr_index)
            if index in self.rviz_ref.keys():
                if index == curr_index:
                    self.rviz_ref[index][0].rviz_manager.startUpdate()
                    rospy.loginfo("starting {}".format(index))
                else:
                    self.rviz_ref[index][0].rviz_manager.stopUpdate()
                    rospy.loginfo("stopping {}".format(index))
