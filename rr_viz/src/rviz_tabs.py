#!/usr/bin/env python
import sys
from PyQt5.QtWidgets import QTabWidget, QVBoxLayout, QWidget

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