#!/usr/bin/env python
from PyQt5.QtWidgets import QTabWidget, QVBoxLayout, QWidget, QHBoxLayout, QLabel, QMessageBox
from PyQt5.QtGui import QPixmap, QFont
from PyQt5.QtCore import Qt, QTimer
from PyQt5 import QtCore

from rviz_tabs import RvizTabs
from ross.rr_interactive_tools import RRInteractiveTools
from ross.esm.environmental_sensing_module import EnvironmentalSensingModule
import managers.file_management as file_management
import subprocess
from sensor_msgs.msg import BatteryState
from rr_custom_msgs.msg import Clock
from std_msgs.msg import Time as RosTime
import rospy

class RRVizTabs(QWidget):
    low_battery_popup = QtCore.pyqtSignal()

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
        
        self.connection_status = QLabel('Connecting')
        self.connection_status.setFont(QFont('Ubuntu', 14, QFont.Bold))
        self.connection_status.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)

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

        # Get Battery levels 
        self.good_voltage = rospy.get_param("/battery/good_voltage", "31")
        self.ok_voltage = rospy.get_param("/battery/ok_voltage", "26")
        self.low_voltage = rospy.get_param("/battery/low_voltage", "22")
        self.battery_state = False

        # Set up battery callbacks
        self.battery_topic = rospy.get_param("/battery/level", "/vesc_driver/battery")
        self.battery_sub = rospy.Subscriber(self.battery_topic, BatteryState, self.battery_cb)
        self.low_battery_popup.connect(self.bat_message_popup)

        # Set up Connection variables
        self.clock_topic = rospy.get_param("/core_clock_sub","/mk3_core/clock")
        self.clock_freq = rospy.get_param("~publish_frequency", 10)
        self.poor_connection_threshold = rospy.get_param("/core_clock_poor_connection", 3)
        self.lost_connection_threshold = rospy.get_param("/core_clock_lost_connection", 5)
        self.last_received_time = RosTime()
        self.first_value = False
        self.get_first_msg = False
        self.latency = 0
        self.read_time = RosTime()
        self.no_time_change = 0
        freq = float(self.clock_freq) * 2
        self.timer_period = 1.0 / freq

        # Connection callbacks
        self.clock_sub = rospy.Subscriber(self.clock_topic, Clock, self.clock_cb)
        self.timer = QTimer.singleShot(self.timer_period, self.clock_timer_cb)


    def clock_timer_cb(self):
        if self.first_value:
            self.last_received_time.data = self.read_time.data
            self.first_value = False            
        elif not self.first_value and self.get_first_msg:
            time_diff = (self.read_time.data - self.last_received_time.data).to_sec()
 
            if time_diff == 0:
                self.no_time_change += 1
                if self.no_time_change > 1:
                    if self.latency < self.lost_connection_threshold + 2:
                        self.latency += 1  
            else:
                self.no_time_change = 0
                if self.latency > 0:
                    self.latency -= 1
            
            if self.latency < self.poor_connection_threshold:
                self.connection_status.setText("Good")
            elif self.latency >= self.lost_connection_threshold:
                self.connection_status.setText("Lost")
            elif self.latency >= self.poor_connection_threshold and self.latency < self.lost_connection_threshold:
                self.connection_status.setText("Poor")
            
            self.last_received_time.data = self.read_time.data   

        freq = float(self.clock_freq) * 2
        self.timer_period = 1.0 / freq
        self.timer = QTimer.singleShot(self.timer_period, self.clock_timer_cb) 
        
    def clock_cb(self, msg):
        self.clock_freq = msg.update_rate
        if self.get_first_msg == False:
            self.first_value = True
            self.get_first_msg = True
        self.read_time.data = msg.data

    def battery_cb(self, msg):
        bat_level = format(msg.voltage, ".1f") + 'V'
            
        if bat_level < self.low_voltage and not self.battery_state:
            self.battery_state = True
            self.low_battery_popup.emit()
        else:
            self.battery_level.setText(bat_level)

    def bat_message_popup(self):
        msg = QMessageBox()
        msg.setText("Battery Low! Please change now!")
        msg.exec_()
