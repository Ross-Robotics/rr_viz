#!/usr/bin/env python
import sys
import signal
import rospy
import rosgraph

from PyQt5.QtWidgets import QApplication, QMainWindow, QMessageBox, QPushButton
from PyQt5 import QtCore, QtGui
from PyQt5.QtCore import QTimer

from rr_viz_tabs import RRVizTabs
from helpers import rr_qt_helper
import managers.file_management as file_management

class RossRoboticsRViz(QMainWindow):
    def __init__(self, parent=None):
        super(QMainWindow, self).__init__(parent)
        self.setWindowTitle("Ross Robotics RViz")
        logo_path = file_management.get_rrviz_resdir() + "/low_level_gui.png"
        self.setWindowIcon(QtGui.QIcon(logo_path))
        self.x = 10
        self.y = 10
        self.width = 1482
        self.height = 868
        self.setGeometry(self.x, self.y, self.width, self.height)

        file_management.check_user_dir()
        
        self.tabWidget = RRVizTabs(self)
        self.setCentralWidget(self.tabWidget)

    def closeEvent(self, event):
        sys.exit(0)

if __name__ == '__main__':
    rospy.init_node("rr_viz")
    app = QApplication(sys.argv)
    main_window = RossRoboticsRViz()
    main_window.show()

    signal.signal(signal.SIGINT, signal.SIG_DFL)
    sys.exit(app.exec_())
