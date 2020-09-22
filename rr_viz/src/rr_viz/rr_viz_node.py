#!/usr/bin/env python
import os
import rospy
import threading
import signal
from functools import partial

from PyQt5 import QtCore, QtGui, QtWidgets, uic
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import pyqtSlot
import managers.file_management as file_management
current_dir = os.path.dirname(os.path.abspath(__file__))
Form, Base = uic.loadUiType(os.path.join(current_dir, "rr_viz.ui"))


class MainWidget(Base, Form):
    def __init__(self, parent=None):
        super(self.__class__, self).__init__(parent)
        self.setupUi(self)
        self.setWindowTitle("RossRobotics RViz")


if __name__ == '__main__':
    import sys
    rospy.init_node("rr_viz")
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle("fusion")
    w = MainWidget()
    w.show()
    # Needed for ctrl+c to work on pyqt applications
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    sys.exit(app.exec_())
