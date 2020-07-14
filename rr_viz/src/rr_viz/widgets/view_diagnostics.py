import os
import rospy
from PyQt5 import QtCore, QtGui, QtWidgets, uic
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import pyqtSlot

# import rviz
from rviz_frame import RViz_frame
import managers.file_management as file_management
from rr_diagnostics_msgs.msg import rr_machine_latency
from rr_diagnostics_msgs.srv import rr_network_latency, rr_network_latencyResponse, rr_network_latencyRequest
import status_machine


current_dir = os.path.dirname(os.path.abspath(__file__))
Form, Base = uic.loadUiType(os.path.join(current_dir, "diagnostics_tab.ui"))
# NOTE: Change this to change what frame config is loaded by this view


class ViewDiagnostics(Base, Form):
    add_machine_signal = QtCore.pyqtSignal(str)

    def __init__(self, parent=None):
        super(self.__class__, self).__init__(parent)
        self.setupUi(self)
        # self.frame.setLayout(QHBoxLayout())
        print("Diagnostics Loaded")
        try:
            rospy.wait_for_service('get_network_latency',
                                   rospy.Duration(secs=3))
        except:
            rospy.loginfo("Failed to connect to get_network_latency")
            self.setEnabled(False)
        else:
            self.diagnostics_service = rospy.ServiceProxy(
                'get_network_latency', rr_network_latency)
            srv_timer = rospy.Timer(rospy.Duration(
                1), self.update_all_machine_statuses)

            self.machine_dict = dict()
            self.add_machine_signal.connect(self.add_machine_slot)

    def __del__(self):
        self.srv_timer.shutdown()
        del(self.srv_timer)

    def update_all_machine_statuses(self, msg):
        request = rr_network_latencyRequest()
        try:
            network_latency = self.diagnostics_service(request)
        except rospy.ServiceException, e:
            print "Service call  to get_network_latency failed: %s" % e
            return

        for machine_latency in network_latency.machine_latency_array:
            if machine_latency.ip in self.machine_dict.keys():
                self.machine_dict[machine_latency.ip].update_frame(
                    machine_latency)
            else:
                self.add_machine_signal.emit(machine_latency.ip)

    def add_machine_slot(self, ip):
        machine_diag_widget = status_machine.StatusMachineWidget(ip)
        self.machine_dict[ip] = machine_diag_widget
        self.frame.addWidget(machine_diag_widget)
