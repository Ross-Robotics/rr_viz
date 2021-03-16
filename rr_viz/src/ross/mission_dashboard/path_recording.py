#!/usr/bin/env python
import sys
from PyQt5.QtWidgets import QVBoxLayout, QHBoxLayout, QWidget, QLabel, QPushButton, QGridLayout
from PyQt5.QtGui import QFont
from PyQt5.QtCore import Qt

class PathRecording(QWidget):
    def __init__(self, parent):
        super(QWidget, self).__init__(parent)

        self.v_layout = QVBoxLayout()

        #Title
        self.title_label = QLabel('Path Recording')
        self.title_label.setFont(QFont('Ubuntu', 11, QFont.Bold))
        self.title_label.setAlignment(Qt.AlignRight)
        self.v_layout.addWidget(self.title_label)

        #Status
        self.h_layout = QHBoxLayout()
        self.status_text_label = QLabel('Status:')
        self.status_text_label.setFont(QFont('Ubuntu', 10, QFont.Bold))
        self.status_label = QLabel('')
        self.h_layout.addWidget(self.status_text_label, 3)
        self.h_layout.addWidget(self.status_label, 7)
        
        self.v_layout.addLayout(self.h_layout)

        #Buttons
        self.grid_layout = QGridLayout()
        self.start_recording_button = QPushButton('Start Recording')
        self.stop_recording_button = QPushButton('Stop Recording')
        self.follow_saved_path_button = QPushButton('Follow Saved Path')
        self.stop_following_button = QPushButton('STOP FOLLOWING')
        self.grid_layout.addWidget(self.start_recording_button, 0, 0)
        self.grid_layout.addWidget(self.stop_recording_button, 0, 1)
        self.grid_layout.addWidget(self.follow_saved_path_button, 1, 0)
        self.grid_layout.addWidget(self.stop_following_button, 1, 1)

        self.v_layout.addLayout(self.grid_layout)

        self.setLayout(self.v_layout)