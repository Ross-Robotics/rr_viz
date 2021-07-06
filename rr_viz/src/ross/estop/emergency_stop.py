#!/usr/bin/env python
from PyQt5.QtWidgets import QVBoxLayout, QWidget, QLabel, QPushButton, QHBoxLayout
from PyQt5.QtGui import QFont
from PyQt5.QtCore import Qt

import rospy
from std_srvs.srv import SetBool, SetBoolRequest
from std_msgs.msg import Bool

class EmergencyStop(QWidget):
    def __init__(self, parent):
        super(QWidget, self).__init__(parent)
        
        self.layout_init()

        # Set up service
        self.eStop_srv_name = '/emergency_stop/trigger_eStop'
        self.eStop_srv = rospy.ServiceProxy(self.eStop_srv_name, SetBool)

        self.eStop_srv_timer = rospy.Timer(rospy.Duration(
            1), self.ping_srv)
        self.eStop_srv_connected = False

        # Set up subscriber
        self.eStop_status_topic_name = '/emergency_stop/status'
        self.eStop_status_sub = rospy.Subscriber(self.eStop_status_topic_name, Bool, self.eStop_status_update)

    def layout_init(self):
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
        self.enable_eStop_button.setEnabled(False)

        self.disable_eStop_button = QPushButton('Reset Emergency Stop')
        self.disable_eStop_button.pressed.connect(self.disable_eStop)
        self.disable_eStop_button.setEnabled(False)

        self.h_layout.addWidget(self.enable_eStop_button)
        self.h_layout.addWidget(self.disable_eStop_button)

        self.v_layout.addLayout(self.h_layout)

        self.setLayout(self.v_layout)

    def ping_srv(self, period):
        if rospy.is_shutdown():
            return False
        try:
            self.eStop_srv.wait_for_service(timeout=rospy.Duration(3.0))
            if not self.eStop_srv_connected:
                rospy.loginfo_throttle(1,'Connected to {}.'.format(self.eStop_srv_name))
            self.eStop_srv_connected = True
            return True
        except:
            rospy.loginfo_throttle(1,
                                   "Lost Connection to {}".format(self.eStop_srv_name) if self.eStop_srv_connected else "Failed to connect to {}".format(self.eStop_srv_name))
            self.eStop_srv_connected = False

        self.layout_estop_enable_btn(False)
        self.layout_estop_disable_btn(False)

    def enable_eStop(self):
        eStop_req = SetBoolRequest()
        eStop_req.data = True
        self.trigger_service(eStop_req)

    def disable_eStop(self):
        eStop_req = SetBoolRequest()
        eStop_req.data = False
        self.trigger_service(eStop_req)

    def trigger_service(self, req):
        try:
            trig_resp = self.eStop_srv.call(req)
            rospy.loginfo(trig_resp.message)
            if not trig_resp.success:
                msg = "Failed to call '" + self.eStop_srv_name + "' service"
                rospy.logerr(msg)
        except:
            msg = "Service '" + self.eStop_srv_name + "' unavailable"
            rospy.logwarn(msg)

    def eStop_status_update(self, msg):
        # This watches a topic created by the Estop script which outputs whether the eStop is 
        # engaged or not. This is so that if the eStop is engaged on one of the controls then
        # all controls know of its engaged and reflect that in their displays. 
        # True = Enabled, False = Disabled
        if msg.data:
            self.layout_estop_enable_btn(False)
            self.layout_estop_disable_btn(True)
        else:
            self.layout_estop_enable_btn(True)
            self.layout_estop_disable_btn(False)
             

    def layout_estop_enable_btn(self, enabled):
        if enabled:
            self.enable_eStop_button.setEnabled(True)
            self.enable_eStop_button.setStyleSheet("color: white;"
                                                    "background-color: red;"
                                                    "font-weight: bold;")
        else:
            self.enable_eStop_button.setEnabled(False)
            self.enable_eStop_button.setStyleSheet("")

    def layout_estop_disable_btn(self, enabled):
        if enabled:
            self.disable_eStop_button.setEnabled(True)
            self.disable_eStop_button.setStyleSheet("color: white;"
                                    "background-color: green;"
                                    "font-weight: bold;")
        else:
            self.disable_eStop_button.setEnabled(False)
            self.disable_eStop_button.setStyleSheet("")   
