#!/usr/bin/env python
import sys
from PyQt5.QtWidgets import QVBoxLayout, QHBoxLayout, QWidget, QLabel, QPushButton, QListWidget, QLineEdit, QMessageBox, QTableWidget, QTableWidgetItem
from PyQt5.QtGui import QFont
from PyQt5.QtCore import Qt
from PyQt5.QtCore import *
from PyQt5 import QtCore
import sys
import telnetlib
import rospy

class ExplosiveAceID(QWidget):
    def __init__(self, parent):
        super(QWidget, self).__init__(parent)

        self.ace_id_hostname = rospy.get_param("~ace_id_hostname", "92.207.233.130")
        self.ace_id_port = rospy.get_param("~ace_id_port", "92.207.233.130")
        self.tn = telnetlib.Telnet()
        self.connected = False

        self.v_layout = QVBoxLayout()

        # Title
        self.title_label = QLabel('ACE-ID')
        self.title_label.setFont(QFont('Ubuntu', 11, QFont.Bold))
        self.title_label.setAlignment(Qt.AlignRight)
        self.v_layout.addWidget(self.title_label)
        # Table
        self.table = QTableWidget()

        self.v_layout.addWidget(self.table)

        # Buttons
        self.h_layout_buttons = QHBoxLayout()

        self.connect_button = QPushButton("Connect")
        self.connect_button.pressed.connect(self.connect)

        self.acquire_button = QPushButton("Acquire")
        self.acquire_button.setEnabled(False)
        self.acquire_button.pressed.connect(self.acquire)

        self.h_layout_buttons.addWidget(self.connect_button)
        self.h_layout_buttons.addWidget(self.acquire_button)

        self.v_layout.addLayout(self.h_layout_buttons)

        self.setLayout(self.v_layout)
        self.timer_period = 500
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._visibility_manager)
        self.timer.start(self.timer_period)

    def _visibility_manager(self):
        if self.connected == True:
            self.acquire_button.setEnabled(True)
        else:
            self.acquire_button.setEnabled(False)
        pass

    def connect(self):
        try:
            self.tn.open(self.ace_id_hostname,self.ace_id_port)
            self.connected = True
        except:
            print("Cannot connect to the ACE-ID host machine")
            self.connected = False

    def acquire(self):
        if self.connected == True:
            self.tn.write("sync results" + "\n")
            print(tn.read_all())
            data = [[1,2,3],[4,5,6],[7,8,9],[10,11,12]]

            x = 0
            while x < len(data):
                self.table.insertRow(x)
                x += 1

            y = 0
            while y < len(data[0]):
                self.table.insertColumn(y)
                y += 1

            column_titles = ['x','y','z']
            self.table.setHorizontalHeaderLabels(column_titles)

            for row in data:
                index = data.index(row)
                self.table.setItem(index,0,QTableWidgetItem(str(row[0])))
                self.table.setItem(index,1,QTableWidgetItem(str(row[1])))
                self.table.setItem(index,2,QTableWidgetItem(str(row[2])))
