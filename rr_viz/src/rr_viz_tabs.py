#!/usr/bin/env python
from PyQt5.QtWidgets import QTabWidget, QVBoxLayout, QWidget, QHBoxLayout, QLabel
from PyQt5.QtGui import QPixmap, QFont
from PyQt5.QtCore import Qt, QTimer

from rviz_tabs import RvizTabs
from ross.rr_interactive_tools import RRInteractiveTools
from ross.esm.environmental_sensing_module import EnvironmentalSensingModule
import managers.file_management as file_management
import subprocess
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Time
import rospy

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

        # Set up rviz screens
        self.rr_interactive_tools_tab.v_layout1.addWidget(RvizTabs(self))

        self.rr_interactive_tools_tab.v_layout1.addWidget(EnvironmentalSensingModule(self))

        # Set up status labels
        self.status_h_layout = QHBoxLayout()
        self.connection_label_text = QLabel('Connection status:')
        self.connection_label_text.setFont(QFont('Ubuntu', 14, QFont.Bold))
        self.connection_label_text.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        
        self.connection_status = QLabel('Connection lost')
        self.connection_status.setFont(QFont('Ubuntu', 14, QFont.Bold))
        self.connection_status.setStyleSheet("color: red")
        self.connection_status.setAlignment(Qt.AlignRight | Qt.AlignVCenter)

        self.battery_label = QLabel('Battery Level:')
        self.battery_label.setFont(QFont('Ubuntu', 14, QFont.Bold))
        self.battery_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)

        self.battery_level = QLabel('N/A')
        self.battery_level.setFont(QFont('Ubuntu', 14, QFont.Bold))
        self.battery_level.setAlignment(Qt.AlignRight | Qt.AlignVCenter)

        self.status_h_layout.addWidget(self.connection_label_text, 3)
        self.status_h_layout.addWidget(self.connection_status, 3)

        self.status_h_layout.addWidget(self.battery_label, 3)
        self.status_h_layout.addWidget(self.battery_level, 1)

        self.rr_interactive_tools_tab.v_layout2.addLayout(self.status_h_layout,1)
     
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

        # Set up topic subscribers
        self.battery_level_sub_name = "/vesc_driver/battery"
        self.battery_level_sub = rospy.Subscriber(self.battery_level_sub_name, BatteryState, self.battery_level_update)

        # Set up core connection variables
        self.core_clock_sub_name ="/core_clock_publisher/clock"
        self.core_clock_sub = rospy.Subscriber(self.core_clock_sub_name, Time, self.core_clock_cb)
        self.last_received_time = Time()

        self.read_time = Time()
        self.no_time_change = 0
        self.latch = 0

        # Get Battery levels 
        self.good_voltage = rospy.get_param("/battery/good_voltage", "31")
        self.ok_voltage = rospy.get_param("/battery/ok_voltage", "26")
        self.low_voltage = rospy.get_param("/battery/low_voltage", "22")

        # Variables required to detect if connected to ROS MASTER
        self.ros_loss_triggered = True
        self.rosMasterIP = '192.168.10.100'
        self.subprocess_command = 'fping -nV -t 50 ' + self.rosMasterIP
        self.fp_status = "alive"  

        # Set up timer 
        self.timer_period = 1000
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.ros_is_up)
        self.timer.start(self.timer_period)

    def core_clock_cb(self, msg):
        self.read_time = msg
        

    def ros_is_up(self):
        if self.last_received_time.data.secs == 0:
            self.last_received_time = self.read_time
            return

        if self.read_time.data.secs - self.last_received_time.data.secs == 0:
            self.no_time_change += 1
        else:
            self.no_time_change = 0

        if(self.no_time_change <= 3):
            self.connection_status.setText("Good")
            self.connection_status.setStyleSheet("color: green")
        elif self.no_time_change > 3 and self.no_time_change <= 5: 
            self.connection_status.setText("Poor")
            self.connection_status.setStyleSheet("color: orange")
        else:
            self.connection_status.setText("Lost")
            self.connection_status.setStyleSheet("color: red")

        self.last_received_time = self.read_time

    def battery_level_update(self, msg):
        bat_level = format(msg.voltage, ".1f") + 'V'
        self.battery_level.setText(bat_level)
        if bat_level >= self.ok_voltage:
            self.battery_level.setStyleSheet("color: green")
        elif bat_level <= self.ok_voltage and bat_level >= self.low_voltage:
            self.battery_level.setStyleSheet("color: orange")
        else:
            self.battery_level.setStyleSheet("color: red")
