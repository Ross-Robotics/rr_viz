#!/usr/bin/env python
import sys
import signal
import rospy
import rosgraph

from PyQt5.QtWidgets import QApplication, QMainWindow, QMessageBox, QPushButton
from PyQt5 import QtCore
from PyQt5.QtCore import QTimer

from rr_viz_tabs import RRVizTabs
from helpers import rr_qt_helper
from fping import FastPing

class RossRoboticsRViz(QMainWindow):
    def __init__(self, parent=None):
        super(QMainWindow, self).__init__(parent)
        self.setWindowTitle("Ross Robotics RViz")
        self.x = 10
        self.y = 10
        self.width = 1482
        self.height = 868
        self.setGeometry(self.x, self.y, self.width, self.height)
        self.tabWidget = RRVizTabs(self)
        self.setCentralWidget(self.tabWidget)

if __name__ == '__main__':
    rospy.init_node("rr_viz")
    app = QApplication(sys.argv)
    main_window = RossRoboticsRViz()
    main_window.show()

    signal.signal(signal.SIGINT, signal.SIG_DFL)
    sys.exit(app.exec_())
