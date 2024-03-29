#!/usr/bin/env python
from PyQt5.QtWidgets import QVBoxLayout, QWidget, QFrame
from PyQt5.QtGui import QFont

from docking import Docking
from path_navigation_tools import PathNavigationTools
from mission_controls import MissionControls
from ..estop.emergency_stop import EmergencyStop

class MissionDashboard(QWidget):
    def __init__(self, parent):
        super(QWidget, self).__init__(parent)

        self.v_layout = QVBoxLayout()

        # Docking set up
        self.docking_widget = Docking(self)

        self.v_layout.addWidget(self.docking_widget, 1)

        self.line = QFrame()
        self.line.setFrameShape(QFrame.HLine)
        self.line.setFrameShadow(QFrame.Sunken)

        self.v_layout.addWidget(self.line)

        # Path navigation tools set up
        self.path_navigation_tools_widget = PathNavigationTools(self)
        self.v_layout.addWidget(self.path_navigation_tools_widget, 4)

        self.line2 = QFrame()
        self.line2.setFrameShape(QFrame.HLine)
        self.line2.setFrameShadow(QFrame.Sunken)

        self.v_layout.addWidget(self.line2)

        # Mission controls set up
        self.mission_controls_widget = MissionControls(self)
        self.v_layout.addWidget(self.mission_controls_widget, 4)       

        self.line3 = QFrame()
        self.line3.setFrameShape(QFrame.HLine)
        self.line3.setFrameShadow(QFrame.Sunken)

        self.v_layout.addWidget(self.line3)

        # Emergency stop set up
        self.emergency_stop_widget = EmergencyStop(self)
        self.v_layout.addWidget(self.emergency_stop_widget, 1)

        self.setLayout(self.v_layout)
