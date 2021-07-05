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
        
        self.connection_status = QLabel('Lost')
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

        # Get Battery levels 
        self.good_voltage = rospy.get_param("/battery/good_voltage", "31")
        self.ok_voltage = rospy.get_param("/battery/ok_voltage", "26")
        self.low_voltage = rospy.get_param("/battery/low_voltage", "22")
        self.battery_state = False

        # Set up topic subscribers
        self.battery_level_sub_name = rospy.get_param("/battery/level", "/vesc_driver/battery")
        self.battery_level_sub = rospy.Subscriber(self.battery_level_sub_name, BatteryState, self.battery_level_update)

        # Set up core connection variables
        self.core_clock_sub_name = rospy.get_param("/core_clock_sub","/clock")
        self.core_clock_sub = rospy.Subscriber(self.core_clock_sub_name, Clock, self.core_clock_cb)
        self.last_received_time = RosTime()
        

        self.read_time = RosTime()
        self.no_time_change = 0
        

        # Core timer
        core_clock_pub_freq = rospy.get_param("~publish_frequency", 10)

        # # To calculate the period for the timer to monitor if the clock cb times out we want the timer to timeout after an 
        # # additional 10%. As this maths is being done as a frequency the 10% needs to be subtracted so that on the following line
        # # the period ends up being 10% longer than the clock publishing
        freq = float(core_clock_pub_freq) * 2
        self.core_clock_timer_dur = rospy.Duration(1.0 / freq)
        self.core_clock_timer = rospy.Timer(self.core_clock_timer_dur, self.core_clock_timer_cb, oneshot = True)
        self.connection_latency = rospy.get_param("/core_clock_latency", "0.1")
        # self.core_clock_timer_trig = False
        
        # Set up timer 

        self.low_battery_popup.connect(self.bat_message_popup)
        self.first_value = False
        self.get_first_msg = False

    def core_clock_timer_cb(self, event):
        self.core_clock_timer = rospy.Timer(self.core_clock_timer_dur, self.core_clock_timer_cb, oneshot = True)
        # rospy.loginfo("Trig")
        if self.first_value:
            # print("timer")
            self.last_received_time.data = self.read_time.data
            # q = self.last_received_time.to_nsec()
            # print(q)
            self.first_value = False            
            return
        elif self.first_value == False and self.get_first_msg:
            time_diff = (self.read_time.data - self.last_received_time.data).to_sec()
            print(time_diff)
            # if(time_diff > self.connection_latency):
            #     print("nooooo")
            # else:
            #     print("yess")

        #     if (self.last_received_time != self.read_time):
            #  x = 
            #  q = x.to_sec()
            #  print(q)   
            
            

            # rospy.loginfo("Trig")
            # print(self.read_time)
            # print(self.last_received_time)
            
            # print(self.read_time.data.nsecs - self.last_received_time.data.nsecs)
            # if self.read_time - self.last_received_time == 0: 
            #     self.connection_status.setText("Lost")
            #     self.connection_status.setStyleSheet("color: red")
            # else:
            #     self.connection_status.setText("Good")
            #     self.connection_status.setStyleSheet("color: green")

            #self.last_received_time = self.read_time
        
    def core_clock_cb(self, msg):
        # rospy.loginfo("MESSAGE")
        self.timer_period = msg.update_rate
        if self.get_first_msg == False:
            # print("first value cb")
            self.first_value = True
            self.get_first_msg = True
        self.read_time.data = msg.data
        print(self.read_time)
       
    # def ros_is_up(self):
    #     if self.last_received_time.data.secs == 0:
    #         self.last_received_time = self.read_time
    #         return

    #     if self.read_time.data.secs - self.last_received_time.data.secs == 0:
    #         self.no_time_change += 1
    #     else:
    #         self.no_time_change = 0

    #     if(self.no_time_change <= 3):
    #         self.connection_status.setText("Good")
    #         self.connection_status.setStyleSheet("color: green")
    #     elif self.no_time_change > 3:
    #         self.connection_status.setText("Poor")
    #         self.connection_status.setStyleSheet("color: orange")
    #     else:
    #         self.connection_status.setText("Lost")
    #         self.connection_status.setStyleSheet("color: red")

    #     self.last_received_time = self.read_time

    def battery_level_update(self, msg):
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
