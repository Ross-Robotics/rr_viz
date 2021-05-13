#!/usr/bin/env python
from PyQt5.QtWidgets import QVBoxLayout, QWidget, QLabel, QPushButton, QHBoxLayout
from PyQt5.QtGui import QFont
from PyQt5.QtCore import Qt

import rospy
from std_srvs.srv import Trigger, TriggerRequest

class EmergencyStop(QWidget):
    def __init__(self, parent):
        super(QWidget, self).__init__(parent)

        self.v_layout = QVBoxLayout()

        # Title
        self.title_label = QLabel('Emergency Stop')
        self.title_label.setFont(QFont('Ubuntu', 11, QFont.Bold))
        self.title_label.setAlignment(Qt.AlignRight)
        self.v_layout.addWidget(self.title_label)
        
        # Buttons
        self.h_layout = QHBoxLayout()
        self.enable_eStop_button = QPushButton('Emergency Stop')
        self.enable_eStop_button.pressed.connect(self.enable_eStop)

        self.disable_eStop_button = QPushButton('Reset Emergency Stop')
        self.disable_eStop_button.pressed.connect(self.disable_eStop)

        self.disable_eStop_button.setEnabled(False)

        self.h_layout.addWidget(self.enable_eStop_button)
        self.h_layout.addWidget(self.disable_eStop_button)

        self.v_layout.addLayout(self.h_layout)

        self.setLayout(self.v_layout)

    def enable_eStop(self):
        self.enable_eStop_button.setEnabled(False)
        self.disable_eStop_button.setEnabled(True)

    def disable_eStop(self):
        self.enable_eStop_button.setEnabled(True)
        self.disable_eStop_button.setEnabled(False)
