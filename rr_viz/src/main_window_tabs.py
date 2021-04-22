#!/usr/bin/env python
import sys
from PyQt5.QtWidgets import QTabWidget, QVBoxLayout, QWidget, QHBoxLayout, QPushButton, QLabel

from rviz_tabs import *
from ross.ross_robotics import *
from webview.rr_webview import *
from webview.rr_webview_tab import *

class MainWindowTabs(QWidget):
    def __init__(self, parent):
        super(QWidget, self).__init__(parent)
        self.main_window_layout = QVBoxLayout(self)

        self.main_window_tabs = QTabWidget()
        self.ross_robotics_tab = QWidget()
        self.verifinder_tab = QWidget()

        self.main_window_tabs.addTab(self.ross_robotics_tab, "Ross Robotics")
        self.main_window_tabs.addTab(self.verifinder_tab, "VeriFinder")

        self.ross_robotics_tab.h_layout = QHBoxLayout(self)
        self.ross_robotics_tab.v_layout1 = QVBoxLayout(self)
        self.ross_robotics_tab.v_layout2 = QVBoxLayout(self)

        self.ross_robotics_tab.v_layout1.addWidget(RvizTabs(self))
        self.title_label = QLabel('Sensor info')
        self.title_label.setFont(QFont('Ubuntu',11,QFont.Bold))
        self.ross_robotics_tab.v_layout1.addWidget(self.title_label)

        self.ross_robotics_tab.v_layout2.addWidget(RossRobotics(self),6)
        self.ross_robotics_tab.v_layout2.addWidget(RRQWebView(self),4,Qt.AlignCenter)

        self.ross_robotics_tab.h_layout.addLayout(self.ross_robotics_tab.v_layout1, 7)
        self.ross_robotics_tab.h_layout.addLayout(self.ross_robotics_tab.v_layout2, 3)

        self.ross_robotics_tab.setLayout(self.ross_robotics_tab.h_layout)


        self.verifinder_tab.layout = QVBoxLayout(self)
        self.verifinder_tab.layout.addWidget(RRQWebViewTab(self),Qt.AlignCenter)
        self.verifinder_tab.setLayout(self.verifinder_tab.layout)

        self.main_window_layout.addWidget(self.main_window_tabs)
        self.setLayout(self.main_window_layout)
