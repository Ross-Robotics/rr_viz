#!/usr/bin/env python
import sys
from PyQt5.QtWidgets import QVBoxLayout, QHBoxLayout, QWidget, QLabel, QPushButton
from PyQt5.QtGui import QFont
from PyQt5.QtCore import Qt

class Docking(QWidget):
    def __init__(self, parent):
        super(QWidget, self).__init__(parent)

        self.v_layout= QVBoxLayout()

        #Title
        self.title_label = QLabel('Docking')
        self.title_label.setFont(QFont('Ubuntu', 11, QFont.Bold))
        self.title_label.setAlignment(Qt.AlignRight)
        self.v_layout.addWidget(self.title_label)

        #Buttons
        self.h_layout = QHBoxLayout()
        self.set_pose_button = QPushButton('Set Docking Pose')
        self.go_to_dock_button = QPushButton('Go to Dock')
        self.h_layout.addWidget(self.set_pose_button)
        self.h_layout.addWidget(self.go_to_dock_button)

        self.v_layout.addLayout(self.h_layout)

        self.setLayout(self.v_layout)