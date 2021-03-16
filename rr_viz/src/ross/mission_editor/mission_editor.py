#!/usr/bin/env python
import sys
from PyQt5.QtWidgets import QVBoxLayout, QHBoxLayout, QWidget, QLabel, QGridLayout, QPushButton, QListWidget, QFrame
from PyQt5.QtGui import QFont
from PyQt5.QtCore import Qt

from ross.mission_editor.pathing.QWaypointListWidget import QWaypointListWidget

class MissionEditor(QWidget):
    def __init__(self, parent):
        super(QWidget, self).__init__(parent)

        self.v_layout = QVBoxLayout(self)

        # Title set up
        self.title_label = QLabel('Mission Editor')
        self.title_label.setFont(QFont('Ubuntu', 11, QFont.Bold))
        self.title_label.setAlignment(Qt.AlignRight)
        self.v_layout.addWidget(self.title_label)

        # Title set up
        self.editor_title_label = QLabel('Editor:')
        self.editor_title_label.setFont(QFont('Ubuntu', 10, QFont.Bold))
        self.v_layout.addWidget(self.editor_title_label)


        # Waypoint list
        # self.waypoint_list = QWidget()
        self.waypoint_list = QWaypointListWidget()
        self.v_layout.addWidget(self.waypoint_list, 5)

        # Buttons
        self.add_here_button = QPushButton('Add Here')
        self.duplicate_button = QPushButton('Duplicate')
        self.delete_button = QPushButton('Delete')
        self.delete_all_button = QPushButton('Delete All')
        self.save_mission_button = QPushButton('Save Mission')
        self.load_mission_button = QPushButton('Load Mission')

        self.button_layout = QGridLayout()
        self.button_layout.addWidget(self.add_here_button, 0, 0)
        self.button_layout.addWidget(self.duplicate_button, 0, 1)
        self.button_layout.addWidget(self.delete_button, 1, 0)
        self.button_layout.addWidget(self.delete_all_button, 1, 1)
        self.button_layout.addWidget(self.save_mission_button, 2, 0)
        self.button_layout.addWidget(self.load_mission_button, 2, 1)
        self.v_layout.addLayout(self.button_layout)

        # Line
        self.line = QFrame()
        self.line.setFrameShape(QFrame.HLine)
        self.line.setFrameShadow(QFrame.Sunken)
        self.v_layout.addWidget(self.line)

        # Send command
        self.command_title = QLabel('Send Command:')
        self.command_title.setFont(QFont('Ubuntu', 10, QFont.Bold))
        self.v_layout.addWidget(self.command_title)

        # Command buttons
        self.h_layout = QHBoxLayout()
        self.go_to_selected_button = QPushButton('Go to Selected')
        self.go_to_all_button = QPushButton('Go to All')
        self.send_mission_button = QPushButton('Send Mission')
        self.h_layout.addWidget(self.go_to_selected_button)
        self.h_layout.addWidget(self.go_to_all_button)
        self.h_layout.addWidget(self.send_mission_button)
        self.v_layout.addLayout(self.h_layout)

        self.setLayout(self.v_layout)
