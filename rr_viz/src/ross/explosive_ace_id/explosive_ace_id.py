#!/usr/bin/env python
import sys
from PyQt5.QtWidgets import QVBoxLayout, QHBoxLayout, QWidget, QLabel, QPushButton, QListWidget, QLineEdit, QMessageBox, QTableWidget, QTableWidgetItem, QHeaderView
from PyQt5.QtGui import QFont
from PyQt5.QtCore import Qt
from PyQt5.QtCore import *
from PyQt5 import QtCore
import sys
import telnetlib
import rospy
import re

class ExplosiveAceID(QWidget):
    def __init__(self, parent):
        super(QWidget, self).__init__(parent)

        self.ace_id_hostname = rospy.get_param("~ace_id_hostname", "192.168.10.130")#92.207.233.130
        self.ace_id_port = rospy.get_param("~ace_id_port", "8007")
        self.tn = telnetlib.Telnet()
        self.telnet_read = ""
        self.results_number = 0
        self.no_results_dict = {
            "No materials found":0.00
        }
        self.results_dict = {}
        self.connected = False
        self.started_acquisition = False

        self.v_layout = QVBoxLayout()

        # Title
        self.title_label = QLabel('ACE-ID')
        self.title_label.setFont(QFont('Ubuntu', 11, QFont.Bold))
        self.title_label.setAlignment(Qt.AlignRight)
        self.v_layout.addWidget(self.title_label)
        # Status
        self.status_label = QLabel('Not connected')
        self.status_label.setFont(QFont('Ubuntu', 10, QFont.Bold))
        self.status_label.setAlignment(Qt.AlignRight)
        self.v_layout.addWidget(self.status_label)

        # Table
        self.table = QTableWidget()
        self.v_layout.addWidget(self.table)
        self._init_table()

        # Buttons
        self.h_layout_buttons = QHBoxLayout()

        self.connect_button = QPushButton("Connect")
        self.connect_button.pressed.connect(self.connect)
        self.connection_attempts = 0

        self.acquire_button = QPushButton("Begin Acquisition")
        self.acquire_button.setEnabled(False)
        self.acquire_button.pressed.connect(self.acquire)

        self.h_layout_buttons.addWidget(self.connect_button)
        self.h_layout_buttons.addWidget(self.acquire_button)

        self.v_layout.addLayout(self.h_layout_buttons)

        self.setLayout(self.v_layout)
        #Timers
        self.visibility_timer_period = 500
        self.visibility_timer = QTimer(self)
        self.visibility_timer.timeout.connect(self._visibility_manager)
        self.visibility_timer.start(self.visibility_timer_period)

        self.acquisition_timer_period = 500
        self.acquisition_timer = QTimer(self)
        self.acquisition_timer.timeout.connect(self._acquisition_timer)

        self.connection_timer_period = 500
        self.connection_timer = QTimer(self)
        self.connection_timer.timeout.connect(self._connection_timer)

        self.results_timer_period = 2000
        self.results_timer = QTimer(self)
        self.results_timer.timeout.connect(self._results_timer)

    def _visibility_manager(self):
        if self.connected == True:
            if self.started_acquisition == False:
                self.acquire_button.setEnabled(True)
            if self.started_acquisition == True:
                self.connect_button.setEnabled(False)
            else:
                self.connect_button.setEnabled(True)
            # if self.started_acquisition == False:
            #     self.status_label.setText('Connected')
        else:
            self.acquire_button.setEnabled(False)
            if self.connection_attempts == 0:
                self.status_label.setText('Not connected')
            if self.connection_attempts == -1:
                # self.status_label.setText('Please connect the Arm and the ACE-ID')
                self.connect_button.setEnabled(True)



    def connect(self):
        try:
            self.tn.close()
        except:
            rospy.loginfo("ACE-ID telnet port cannot be closed.")
        try:
            self.connection_attempts = 1
            self.tn.open(self.ace_id_hostname,self.ace_id_port,timeout=1)
            self.connection_attempts = 0
            self.check_aceid_connection()
        except:
            rospy.loginfo("Please connect the Arm and the ACE-ID")
            self.status_label.setText('Please connect the Arm and the ACE-ID')
            self.connection_attempts = -1
            self.connected = False
            self.started_acquisition = False

    def check_aceid_connection(self):
        try:
            self.tn.write("id" + "\n")
            self.connection_timer.start(self.connection_timer_period)
        except:
            rospy.loginfo("ACE-ID telnet port cannot be written to check connection.")
            self.status_label.setText('Plese connect the ACE-ID to the Arm')
            self.connection_attempts = -1
            self.connected = False
            self.started_acquisition = False


    def _connection_timer(self):
        try:
            self.connection_attempts =  self.connection_attempts + 1
            rospy.loginfo("ACE-ID Waiting for ID and firmware version.")
            self.status_label.setText('Port open, trying to connect.')
            self.telnet_read = self.telnet_read + self.tn.read_very_eager()
            self.telnet_read = self.telnet_read.replace("\n",'')
            substrings = self.telnet_read.split("\r")
            for i in range(len(substrings)):
                if substrings[i].find("SERIAL") > 1:
                    rospy.loginfo("ACE-ID found.")
                    self.telnet_read = ""
                    self.connected = True
                    self.connection_timer.stop()
                    self.connection_attempts = 0
                    self.status_label.setText('Connected')

            if self.connection_attempts > 5:
                self.connection_timer.stop()
                self.connected = False
                self.telnet_read = ""
                rospy.loginfo("Port open but ACE-ID Not found.")
                self.status_label.setText('Plese connect the ACE-ID to the Arm')
        except:
            rospy.loginfo("ACE-ID telnet port cannot be read to validate connection.")
            self.telnet_read = ""
            self.connection_timer.stop()
            self.connection_attempts = -1
            self.status_label.setText('Please connect the ACE-ID')
            self.connected = False
            self.started_acquisition = False



    def _acquisition_timer(self):
        try:
            self.started_acquisition = True
            rospy.loginfo("ACE-ID Waiting for acquisition.")
            self.status_label.setText('ACE-ID acquiring data.')
            self.telnet_read = self.telnet_read + self.tn.read_very_eager()
            self.telnet_read = self.telnet_read.replace("\n",'')
            substrings = self.telnet_read.split("\r")
            for i in range(len(substrings)):
                if substrings[i] == "DONE":
                    rospy.loginfo("ACE-ID Acquisition FINISHED.")
                    self.telnet_read = ""
                    self.acquisition_timer.stop()
                    self.get_results()
        except:
            rospy.loginfo("ACE-ID telnet port cannot be read to acquire data.")
            self.telnet_read = ""
            self.acquisition_timer.stop()
            self.connection_attempts = -1
            self.status_label.setText('Plese connect the ACE-ID to the Arm')
            self.connected = False
            self.started_acquisition = False

    def get_results(self):
        try:
            self.tn.write("sync results" + "\n")
            self.results_timer.start(self.results_timer_period)
        except:
            rospy.loginfo("ACE-ID telnet port cannot be written to get results.")
            self.telnet_read = ""
            self.acquisition_timer.stop()
            self.connection_attempts = -1
            self.status_label.setText('Plese connect the ACE-ID to the Arm')
            self.connected = False
            self.started_acquisition = False

    def _results_timer(self):
        try:
            rospy.loginfo("ACE-ID Waiting for results.")
            self.telnet_read = self.telnet_read + self.tn.read_until('\0', timeout=0.5)
            self.started_acquisition = False
            print(self.telnet_read)
            got_materials = False
            for line in self.telnet_read.split("\n"):
                if "Quality" in line:
                    rospy.loginfo("ACE-ID Results received.")
                    got_materials = True
                    result_line = line.strip()
                    result_line = result_line.split("] = ")[1].split("] [Contribution")[0]
                    material = result_line.split(" [Quality=")[0]
                    quality = result_line.split(" [Quality=")[1]
                    self.results_dict[material] = quality
                    self.results_number = self.results_number + 1
                    self.results_timer.stop()
                    self.fill_table()
                if "returned" in line:
                    number_of_materials = line.split("returned ")[1]
                    got_materials = False
            if not got_materials:
                self.results_number = 0
                rospy.loginfo("ACE-ID No materials found.")
                self.results_timer.stop()
                self.fill_table()
        except:
            rospy.loginfo("ACE-ID telnet port cannot be read to get results.")
            self.telnet_read = ""
            self.results_timer.stop()
            self.connection_attempts = -1
            self.status_label.setText('Plese connect the ACE-ID to the Arm')
            self.connected = False
            self.started_acquisition = False




    def _results_parser(self):
        example_string = """>sync results

Getting search results...

  tsGetComponents returned 2

  [ 0] = Toluene [Quality=0.96] [Contribution=0.00]

         UCID = 108883, CAS Text = 108-88-3

         Entry 590 from Library 'ACE-ID_Std.lib'

  [ 1] = Acetonitrile [Quality=0.24] [Contribution=0.00]

         UCID = 75058, CAS Text = 75-05-8

         Entry 106 from Library 'ACE-ID_Std.lib'

 """
        for a in example_string:
            print(a,ord(a))
        for line in example_string.split("\n"):
            if "Quality" in line:
                result_line = line.strip()
                result_line = result_line.split("] = ")[1].split("] [Contribution")[0]
                material = result_line.split(" [Quality=")[0]
                quality = result_line.split(" [Quality=")[1]
                self.results_dict[material] = quality
                self.results_number = self.results_number + 1

    def fill_table(self):
        if self.results_number == 0:
            self.results_dict = self.no_results_dict
        for x in self.results_dict.keys():
            row_count = self.table.rowCount()
            self.table.insertRow(self.table.rowCount())
            material_item = QTableWidgetItem(x)
            material_item.setTextAlignment(Qt.AlignCenter)
            material_item.setFlags(QtCore.Qt.ItemIsSelectable | QtCore.Qt.ItemIsEnabled )
            quality_item = QTableWidgetItem(str(self.results_dict[x]))
            quality_item.setTextAlignment(Qt.AlignCenter)
            quality_item.setFlags(QtCore.Qt.ItemIsSelectable | QtCore.Qt.ItemIsEnabled )
            self.table.setItem(row_count,0,material_item)
            self.table.setItem(row_count,1,quality_item)
        self.results_number = 0
        self.results_dict = {}
        self.status_label.setText('Connected')

    def clear_table(self):
        while self.table.rowCount() > 0:
            self.table.removeRow(self.table.rowCount()-1)

    def _init_table(self):
        y = 0
        while y < 2:
            self.table.insertColumn(y)
            y += 1

        column_titles = ['Material','Quality']
        self.table.setHorizontalHeaderLabels(column_titles)
        header = self.table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.Stretch)
        header.setSectionResizeMode(1, QHeaderView.Stretch)

    def acquire(self):
        if self.connected == True:
            if self.started_acquisition == False:
                try:
                    self.tn.write("RC Start" + "\n")
                    self.started_acquisition = True
                    self.acquisition_timer.start(self.acquisition_timer_period)
                    self.acquire_button.setEnabled(False)
                    self.connect_button.setEnabled(False)
                except:
                    rospy.loginfo("Cant begin acquisition because ACE-ID not connected")
                    self.status_label.setText('Plese connect the ACE-ID to the Arm')
                    self.connected == False
                    self.started_acquisition = False

            self.clear_table()
