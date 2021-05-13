#!/usr/bin/env python
import sys
from PyQt5.QtWidgets import QTabWidget, QVBoxLayout, QWidget, QApplication
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QIcon, QCursor
import managers.file_management as file_management

from rviz_frame import *

DEFAULT_RVIZ_CONFIG = "rr_viz_conf.rviz"
CAMERA_360_RVIZ_CONFIG = "rviz_textured_sphere.rviz"

class RvizTabs(QWidget):
    def __init__(self, parent):
        super(QWidget, self).__init__(parent)
        self.rviz_frame = RViz_frame(self)
        self.layout = QVBoxLayout()

        self.tabs = QTabWidget()
        self.default_tab = QWidget()
        self.cam_360_tab = QWidget()

        self.tabs.addTab(self.default_tab, "RViz")
        self.tabs.addTab(self.cam_360_tab, "360 Camera")

        self.default_tab.layout = QVBoxLayout(self)
        self.cam_360_tab.layout = QVBoxLayout(self)

        # Default rviz tab set up
        self.rviz_frame.set_rviz_config(DEFAULT_RVIZ_CONFIG)
        self.rviz_return = self.rviz_frame.load_rviz_frame(hide_menu=False, hide_status=False, splash="")
        self.default_tab.layout.addWidget(self.rviz_return)
        self.default_tab.setLayout(self.default_tab.layout)

        # 360 Camera tab set up
        self.rviz_frame.set_rviz_config(CAMERA_360_RVIZ_CONFIG)
        self.rviz_return = self.rviz_frame.load_rviz_frame(hide_menu=True, hide_status=True, splash="")
        self.cam_360_tab.layout.addWidget(self.rviz_return)
        self.cam_360_tab.setLayout(self.cam_360_tab.layout)

        self.layout.addWidget(self.tabs)
        self.setLayout(self.layout)

        # Get png files for spawn waypoint
        self.spawn_waypoint_png = file_management.get_rrviz_resdir() + "/spawn_waypoint.png"
        self.spawn_waypoint_cursor_png = file_management.get_rrviz_resdir() + "/cursor.png"
        self.cursor_pixmap = QtGui.QPixmap(self.spawn_waypoint_cursor_png)  

        self.pose_estimates = []
        self.timer = QTimer(self)
        for button in self.findChildren(QtWidgets.QToolButton):
            if "Estimate" in button.text():
                self.pose_estimates.append(button)

        # Workaround for the refresh that rviz does
        self.timer.timeout.connect(self.rr_viz_visual_overrides)
        self.timer.start(100)

    def rr_viz_visual_overrides(self):
        try:
            self.pose_estimates[3].setText("Set Pose Estimate")
            self.pose_estimates[4].setText("Spawn Waypoint")
            
            self.pose_estimates[4].setIcon(QIcon(self.spawn_waypoint_png))

            if(self.pose_estimates[4].isChecked()):
                QApplication.setOverrideCursor(QCursor(self.cursor_pixmap))
            else:
                QApplication.restoreOverrideCursor()
        except:
            pass
