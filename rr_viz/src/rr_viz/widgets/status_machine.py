import os
import rospy
import rospkg
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from functools import partial
from PyQt5 import QtGui, QtWidgets, uic, QtCore
from PyQt5.QtWidgets import QApplication, QWidget, QInputDialog, QLineEdit, QFileDialog, QMessageBox, QListWidgetItem
from rr_diagnostics_msgs.msg import rr_machine_latency
import node_status

current_dir = os.path.dirname(os.path.abspath(__file__))
Form, Base = uic.loadUiType(os.path.join(current_dir, "status_machine.ui"))


class StatusMachineWidget(Base, Form):
    update_frame_signal = QtCore.pyqtSignal(rr_machine_latency)
    add_node_signal = QtCore.pyqtSignal(str)
    remove_node_signal = QtCore.pyqtSignal(str)

    def __init__(self, name, parent=None):
        super(self.__class__, self).__init__(parent)
        self.setupUi(self)
        print("StatusMachineWidget created")
        self.nameLbl.setText(name)
        self.column = self.findChildren(QtWidgets.QFrame, "column")[0]
        self.namebox = self.findChildren(QtWidgets.QFrame, "namebox")[0]
        self.namebox.setStyleSheet(''' border: 0px ;''')
        self.update_frame_signal.connect(self.update_frame_slot)
        self.add_node_signal.connect(self.add_node_status_slot)
        self.node_dict = dict()

    def add_node_status(self, name):
        self.add_node_signal.emit(name)

    def remove_node_status(self, name):
        # print("remove")
        self.remove_node_signal.emit(name)

    def add_node_status_slot(self, name):
        nd_st = node_status.NodeStatusWidget(name)
        self.node_dict[name] = nd_st
        self.column.layout().insertWidget(-1, nd_st)

    def remove_node_status_slot(self, name):
        self.column.layout().removeWidget(self.node_dict[name])
        self.node_dict[name].deleteLater()
        del(self.node_dict[name])

    def update_frame(self, rr_machine_latency_msg):
        self.update_frame_signal.emit(rr_machine_latency_msg)

    def update_frame_slot(self, rr_machine_latency_msg):
        self.latencyLbl.setText("avg/max/min/drop% : {}/{}/{}/{}".format(
            round(rr_machine_latency_msg.avg_ms, 3),
            round(rr_machine_latency_msg.max_ms, 3),
            round(rr_machine_latency_msg.min_ms, 3),
            round(rr_machine_latency_msg.drop_rate, 3)
        ))
        # 50,20
        if rr_machine_latency_msg.avg_ms > 20 or rr_machine_latency_msg.max_ms > 100 or rr_machine_latency_msg.drop_rate > 0.2:
            self.column.setStyleSheet(
                ''' #column{border: 1.5px solid Firebrick;}''')
        elif rr_machine_latency_msg.avg_ms > 10 or rr_machine_latency_msg.max_ms > 50 or rr_machine_latency_msg.drop_rate > 0.05:
            self.column.setStyleSheet(
                ''' #column{border: 1.5px solid Tomato;}''')
        else:
            self.column.setStyleSheet(
                ''' #column{border: 1.5px solid Silver;}''')

        for nodename in rr_machine_latency_msg.nodes:
            if nodename not in self.node_dict.keys():
                self.add_node_status(nodename)

        for nodename in self.node_dict.keys():
            if nodename not in rr_machine_latency_msg.nodes:
                self.remove_node_status(nodename)
