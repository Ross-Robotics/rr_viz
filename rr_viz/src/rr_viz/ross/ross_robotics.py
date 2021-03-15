#!/usr/bin/env python
import sys

from PyQt5.QtWidgets import QTabWidget, QVBoxLayout, QWidget, QHBoxLayout, QPushButton

from rviz_tabs import *
from mission_editor.mission_editor import *
from mission_dashboard.mission_dashboard import *
from slam.slam_supervisor import *

class RossRobotics(QWidget):
    def __init__(self, parent):
        super(QWidget, self).__init__(parent)
        self.layout = QVBoxLayout()

        self.tabs = QTabWidget()
        self.mission_dashboard_tab = QWidget()
        self.mission_editor_tab = QWidget()
        #self.diagnostics_tab = QWidget()
        self.slam_supervisor_tab = QWidget()

        self.tabs.addTab(self.mission_editor_tab, "Mission Editor")
        self.tabs.addTab(self.slam_supervisor_tab, "Slam Supervisor")
        self.tabs.addTab(self.mission_dashboard_tab, "Mission Dashboard")
        #self.tabs.addTab(self.diagnostics_tab, "Diagnostics")
        
        self.mission_editor_tab.layout = QVBoxLayout()
        self.mission_editor_tab.layout.addWidget(MissionEditor(self))
        self.mission_editor_tab.setLayout(self.mission_editor_tab.layout)

        self.slam_supervisor_tab.layout = QVBoxLayout()
        self.slam_supervisor_tab.layout.addWidget(SlamSupervisor(self))
        self.slam_supervisor_tab.setLayout(self.slam_supervisor_tab.layout)

        self.mission_dashboard_tab.layout = QVBoxLayout()
        self.mission_dashboard_tab.layout.addWidget(MissionDashboard(self))
        self.mission_dashboard_tab.setLayout(self.mission_dashboard_tab.layout)



        self.layout.addWidget(self.tabs)
        self.setLayout(self.layout)

        
