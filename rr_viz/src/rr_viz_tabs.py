#!/usr/bin/env python
from PyQt5.QtWidgets import QTabWidget, QVBoxLayout, QWidget, QHBoxLayout, QLabel
from PyQt5.QtGui import QPixmap, QFont
from PyQt5.QtCore import Qt, QTimer

from rviz_tabs import RvizTabs
from ross.rr_interactive_tools import RRInteractiveTools
import managers.file_management as file_management
import subprocess

class RRVizTabs(QWidget):
    def __init__(self, parent):
        super(QWidget, self).__init__(parent)
        self.main_window_layout = QVBoxLayout(self)

        self.main_window_tabs = QTabWidget()
        self.rr_interactive_tools_tab = QWidget()
        
        self.main_window_tabs.addTab(self.rr_interactive_tools_tab, "Ross Robotics")
        
        self.rr_interactive_tools_tab.h_layout = QHBoxLayout(self)
        self.rr_interactive_tools_tab.v_layout1 = QVBoxLayout(self)
        self.rr_interactive_tools_tab.v_layout2 = QVBoxLayout(self)

        self.rr_interactive_tools_tab.v_layout1.addWidget(RvizTabs(self))

        # Set up Connection status labels
        self.connection_h_layout = QHBoxLayout()
        self.connection_label_text = QLabel('Connection status:')
        self.connection_label_text.setFont(QFont('Ubuntu', 14, QFont.Bold))
        self.connection_label_text.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        
        self.connection_status = QLabel('Connection lost')
        self.connection_status.setFont(QFont('Ubuntu', 14, QFont.Bold))
        self.connection_status.setStyleSheet("color: red")
        self.connection_status.setAlignment(Qt.AlignRight | Qt.AlignVCenter)

        self.connection_h_layout.addWidget(self.connection_label_text, 7)
        self.connection_h_layout.addWidget(self.connection_status, 3)
        self.rr_interactive_tools_tab.v_layout2.addLayout(self.connection_h_layout,1)
     
        # Add rr interactive tools to layout
        self.rr_interactive_tools_tab.v_layout2.addWidget(RRInteractiveTools(self),7)

        # Set up logo
        self.logo_label = QLabel(self)
        logo_path = file_management.get_rrviz_resdir() + "/logo.png"
        logo_pixmap = QPixmap(logo_path)
        logo_scaled = logo_pixmap.scaledToWidth(300)
        self.logo_label.setPixmap(logo_scaled)
        
        self.logo_label.setAlignment(Qt.AlignCenter | Qt.AlignVCenter)

        self.rr_interactive_tools_tab.v_layout2.addWidget(self.logo_label, 2)

        # Add vertical layouts to the horizontal layout
        self.rr_interactive_tools_tab.h_layout.addLayout(self.rr_interactive_tools_tab.v_layout1, 7)
        self.rr_interactive_tools_tab.h_layout.addLayout(self.rr_interactive_tools_tab.v_layout2, 3)

        self.rr_interactive_tools_tab.setLayout(self.rr_interactive_tools_tab.h_layout)

        self.main_window_layout.addWidget(self.main_window_tabs)
        self.setLayout(self.main_window_layout)

        # Variables required to detect if connected to ROS MASTER
        self.ros_loss_triggered = True
        self.rosMasterIP = '192.168.10.100'
        self.subprocess_command = 'fping -nV -t 50 ' + self.rosMasterIP
        self.fp_status = "alive"  

        # Set up timer 
        self.timer_period = 1500
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.ros_is_up)
        self.timer.start(self.timer_period)

    def ros_is_up(self):
        process = subprocess.Popen(self.subprocess_command, shell=True, stdout=subprocess.PIPE)
        stdout = process.communicate()[0]

        if not self.fp_status in stdout and not self.ros_loss_triggered:
            self.connection_status.setText("Connection lost")
            self.connection_status.setStyleSheet("color: red")
            self.ros_loss_triggered = True
        elif self.fp_status in stdout and self.ros_loss_triggered:
            self.connection_status.setText("Connected")
            self.connection_status.setStyleSheet("color: green")
            self.ros_loss_triggered = False
