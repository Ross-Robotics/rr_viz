#!/usr/bin/env python
from PyQt5.QtWidgets import QTabWidget, QVBoxLayout, QWidget, QHBoxLayout, QLabel, QMessageBox
from PyQt5.QtGui import QPixmap, QFont
from PyQt5.QtCore import Qt, QTimer
from PyQt5 import QtCore

from rviz_tabs import RvizTabs
from webview.rr_webview import *
from webview.rr_webview_tab import *
from ross.rr_interactive_tools import RRInteractiveTools
from ross.esm.environmental_sensing_module import EnvironmentalSensingModule
import managers.file_management as file_management
import subprocess
from sensor_msgs.msg import BatteryState
from rr_custom_msgs.msg import CommsData, Ping
import rospy

class RRVizTabs(QWidget):
    low_battery_popup = QtCore.pyqtSignal()

    def __init__(self, parent):
        super(QWidget, self).__init__(parent)
        self.main_window_layout = QVBoxLayout(self)

        self.main_window_tabs = QTabWidget()
        self.rr_interactive_tools_tab = QWidget()
        self.verifinder_tab = QWidget()

        self.main_window_tabs.addTab(self.rr_interactive_tools_tab, "Ross Robotics")
        self.main_window_tabs.addTab(self.verifinder_tab, "VeriFinder")

        self.rr_interactive_tools_tab.h_layout = QHBoxLayout(self)
        self.rr_interactive_tools_tab.v_layout1 = QVBoxLayout(self)
        self.rr_interactive_tools_tab.v_layout2 = QVBoxLayout(self)

        # Set up rviz screens
        self.rr_interactive_tools_tab.v_layout1.addWidget(RvizTabs(self))

        self.rr_interactive_tools_tab.v_layout1.addWidget(EnvironmentalSensingModule(self))

        # Set up status labels
        self.status_h_layout = QHBoxLayout()
        self.signal_strength_label = QLabel('Signal Strength:')
        self.signal_strength_label.setFont(QFont('Ubuntu', 11, QFont.Bold))
        self.signal_strength_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        
        self.signal_strength = QLabel('Connecting')
        self.signal_strength.setFont(QFont('Ubuntu', 11, QFont.Bold))
        self.signal_strength.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)

        self.latency_label = QLabel('Latency:')
        self.latency_label.setFont(QFont('Ubuntu', 11, QFont.Bold))
        self.latency_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        
        self.latency = QLabel('0')
        self.latency.setFont(QFont('Ubuntu', 11, QFont.Bold))
        self.latency.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)

        self.battery_label = QLabel('Battery Level:')
        self.battery_label.setFont(QFont('Ubuntu', 11, QFont.Bold))
        self.battery_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)

        self.battery_level = QLabel('N/A')
        self.battery_level.setFont(QFont('Ubuntu', 11, QFont.Bold))
        self.battery_level.setAlignment(Qt.AlignRight | Qt.AlignVCenter)

        self.status_h_layout.addWidget(self.signal_strength_label, 2)
        self.status_h_layout.addWidget(self.signal_strength, 2)

        self.status_h_layout.addWidget(self.latency_label, 2)
        self.status_h_layout.addWidget(self.latency, 2)

        self.status_h_layout.addWidget(self.battery_label, 1)
        self.status_h_layout.addWidget(self.battery_level, 1)

        self.rr_interactive_tools_tab.v_layout2.addLayout(self.status_h_layout,1)

        # Add rr interactive tools to layout
        self.rr_interactive_tools_tab.v_layout2.addWidget(RRInteractiveTools(self),5)
        self.rr_interactive_tools_tab.v_layout2.addWidget(RRQWebView(self),4, Qt.AlignCenter)

        # Add vertical layouts to the horizontal layout
        self.rr_interactive_tools_tab.h_layout.addLayout(self.rr_interactive_tools_tab.v_layout1, 7)
        self.rr_interactive_tools_tab.h_layout.addLayout(self.rr_interactive_tools_tab.v_layout2, 3)

        self.rr_interactive_tools_tab.setLayout(self.rr_interactive_tools_tab.h_layout)

        self.verifinder_tab.layout = QVBoxLayout(self)
        self.verifinder_tab.layout.addWidget(RRQWebViewTab(self), Qt.AlignCenter)
        self.verifinder_tab.setLayout(self.verifinder_tab.layout)

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

        # Set up signal callback
        self.comms_topic = rospy.get_param("/comms_topic","/rr_ocu/comms_data")
        self.comms_sub = rospy.Subscriber(self.comms_topic, CommsData, self.comms_cb)

        # Set up latency callback
        self.latency_topic = rospy.get_param("latency_topic", "/mk3_core_connection/ping_data")
        self.latency_sub = rospy.Subscriber(self.latency_topic, Ping, self.latency_cb)

    def comms_cb(self, msg):
        self.signal_strength.setText(str(msg.signal) + " dB")

    def latency_cb(self, msg):
        self.latency.setText(format(msg.latency, ".2f") + " ms")

    def battery_cb(self, msg):
        bat_level = format(msg.voltage, ".1f") + ' V'

        if bat_level < self.low_voltage and not self.battery_state:
            self.battery_state = True
            self.low_battery_popup.emit()
        else:
            self.battery_level.setText(bat_level)

    def bat_message_popup(self):
        msg = QMessageBox()
        msg.setText("Battery Low! Please change now!")
        msg.exec_()
