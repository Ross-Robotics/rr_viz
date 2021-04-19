#!/usr/bin/env python
import sys
from PyQt5.QtWidgets import QVBoxLayout, QWidget, QFrame, QLabel
from PyQt5.QtGui import QFont

from docking import *
from path_navigation_tools import *
from mission_controls import *

class MissionDashboard(QWidget):
    def __init__(self, parent):
            super(QWidget, self).__init__(parent)

            self.v_layout = QVBoxLayout()

            #Docking set up
            self.docking_widget = Docking(self)

            self.v_layout.addWidget(self.docking_widget, 2)

            self.line = QFrame()
            self.line.setFrameShape(QFrame.HLine)
            self.line.setFrameShadow(QFrame.Sunken)

            self.v_layout.addWidget(self.line)

            self.path_navigation_tools_widget = PathNavigationTools(self)
            self.v_layout.addWidget(self.path_navigation_tools_widget,4)


            self.line2 = QFrame()
            self.line2.setFrameShape(QFrame.HLine)
            self.line2.setFrameShadow(QFrame.Sunken)
            self.v_layout.addWidget(self.line2)

            self.mission_controls_widget = MissionControls(self)
            self.v_layout.addWidget(self.mission_controls_widget, 4)       

            self.setLayout(self.v_layout)
