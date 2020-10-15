#!/usr/bin/env python
import os
import rospy
import rosgraph
import threading
import signal
from functools import partial

from PyQt5 import QtCore, QtGui, QtWidgets, uic
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import QApplication, QWidget, QInputDialog, QLineEdit, QFileDialog, QMessageBox
from PyQt5.QtCore import pyqtSlot
import managers.file_management as file_management
from helpers import rr_qt_helper
current_dir = os.path.dirname(os.path.abspath(__file__))
Form, Base = uic.loadUiType(os.path.join(current_dir, "rr_viz.ui"))


class MainWidget(Base, Form):
    ross_loss_signal = QtCore.pyqtSignal(bool)

    def __init__(self, parent=None):
        super(self.__class__, self).__init__(parent)
        self.setupUi(self)
        self.setWindowTitle("RossRobotics RViz")
        self.ross_loss_signal.connect(self.ros_loss_slot)
        self.ros_loss_triggered = False

    def ros_is_up(self):
        try:
            rosgraph.Master('/rosout').getPid()
            return True
        except Exception:
            return False

    def ros_loss_slot(self, flag):
        if not flag and not self.ros_loss_triggered:
            self.ros_loss_triggered = True
            self.hide()
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Warning)
            msg.setText(
                "RR_VIZ lost connection to the ROS MASTER. Please restart RR_VIZ")
            msg.setWindowTitle("Connection Lost")
            msg.setStandardButtons(QMessageBox.Ok)
            # msg.buttonClicked.connect(msgbtn)
            msg.exec_()
            sys.exit()


if __name__ == '__main__':
    import sys
    rospy.init_node("rr_viz")
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle("fusion")
    w = MainWidget()
    w.show()
    ros_state_checker = rr_qt_helper.StateCheckerTimer(
        w.ros_is_up, w.ross_loss_signal, Hz=1)
    ros_state_checker.start()
    # Needed for ctrl+c to work on pyqt applications
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    sys.exit(app.exec_())
