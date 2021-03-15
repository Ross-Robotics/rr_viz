#!/usr/bin/env python
import sys
from PyQt5.QtWidgets import QVBoxLayout, QHBoxLayout, QWidget, QLabel, QPushButton, QGridLayout
from PyQt5.QtGui import QFont
from PyQt5.QtCore import Qt

class MissionControls(QWidget):
    def __init__(self, parent):
        super(QWidget, self).__init__(parent)

        self.v_layout = QVBoxLayout()

        #Title
        self.title_label = QLabel('Mission Controls')
        self.title_label.setFont(QFont('Ubuntu', 11, QFont.Bold))
        self.title_label.setAlignment(Qt.AlignRight)
        self.v_layout.addWidget(self.title_label)

        #Status
        self.h_layout = QHBoxLayout()
        self.status_text_label = QLabel('Status:')
        self.status_text_label.setFont(QFont('Ubuntu', 10, QFont.Bold))
        self.status_label = QLabel('N/A')
        self.h_layout.addWidget(self.status_text_label, 2)
        self.h_layout.addWidget(self.status_label, 8)
        
        self.v_layout.addLayout(self.h_layout)

        #buttons
        self.grid_layout = QGridLayout()
        self.start_mission_button = QPushButton('Start Mission')
        self.next_waypoint_button = QPushButton('Next Waypoint')
        self.stop_mission_button = QPushButton('Stop Mission')
        self.set_active_button = QPushButton('Set Active')
        self.emergancy_stop_button = QPushButton('EMERGENCY STOP')
        self.eStop_resume_button = QPushButton('eStop Resume')
        self.grid_layout.addWidget(self.start_mission_button, 0, 0)
        self.grid_layout.addWidget(self.next_waypoint_button, 0, 1)
        self.grid_layout.addWidget(self.stop_mission_button, 0, 2)
        self.grid_layout.addWidget(self.set_active_button, 1, 0)
        self.grid_layout.addWidget(self.emergancy_stop_button, 1, 1)
        self.grid_layout.addWidget(self.eStop_resume_button, 1, 2)

        self.v_layout.addLayout(self.grid_layout)

        self.setLayout(self.v_layout)