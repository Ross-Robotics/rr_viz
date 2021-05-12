#!/usr/bin/env python
from PyQt5.QtWidgets import QTabWidget, QVBoxLayout, QWidget

from rviz_tabs import RvizTabs
from mission_editor.mission_editor import MissionEditor
from mission_dashboard.mission_dashboard import MissionDashboard
from slam.slam_supervisor import SlamSupervisor
from explosive_ace_id.explosive_ace_id import ExplosiveAceID


class RRInteractiveTools(QWidget):
    def __init__(self, parent):
        super(QWidget, self).__init__(parent)
        self.layout = QVBoxLayout()

        self.tabs = QTabWidget()
        self.mission_dashboard_tab = QWidget()
        self.mission_editor_tab = QWidget()
        self.slam_supervisor_tab = QWidget()
        self.explosive_sensor_tab = QWidget()

        self.tabs.addTab(self.mission_editor_tab, "Mission Editor")
        self.tabs.addTab(self.slam_supervisor_tab, "Slam Supervisor")
        self.tabs.addTab(self.mission_dashboard_tab, "Mission Dashboard")
        self.tabs.addTab(self.explosive_sensor_tab, "ACE-ID")
        
        self.mission_editor_tab.layout = QVBoxLayout()
        self.mission_editor_tab.layout.addWidget(MissionEditor(self))
        self.mission_editor_tab.setLayout(self.mission_editor_tab.layout)

        self.slam_supervisor_tab.layout = QVBoxLayout()
        self.slam_supervisor_tab.layout.addWidget(SlamSupervisor(self))
        self.slam_supervisor_tab.setLayout(self.slam_supervisor_tab.layout)

        self.mission_dashboard_tab.layout = QVBoxLayout()
        self.mission_dashboard_tab.layout.addWidget(MissionDashboard(self))
        self.mission_dashboard_tab.setLayout(self.mission_dashboard_tab.layout)

        self.explosive_sensor_tab.layout = QVBoxLayout()
        self.explosive_sensor_tab.layout.addWidget(ExplosiveAceID(self))
        self.explosive_sensor_tab.setLayout(self.explosive_sensor_tab.layout)

        self.layout.addWidget(self.tabs)
        self.setLayout(self.layout)
