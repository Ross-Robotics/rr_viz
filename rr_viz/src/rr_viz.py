#!/usr/bin/env python
import sys
import signal
import rospy
import rosgraph

from PyQt5.QtWidgets import QApplication, QMainWindow

from rr_viz_tabs import RRVizTabs
from helpers import rr_qt_helper

class RossRoboticsRViz(QMainWindow):
    ross_loss_signal = QtCore.pyqtSignal(bool)

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
            msg.exec_()
            sys.exit()


if __name__ == '__main__':
    rospy.init_node("rr_viz")
    app = QApplication(sys.argv)
    main_window = RossRoboticsRViz()
    main_window.show()
    ros_state_checker = rr_qt_helper.StateCheckerTimer(
        main_window.ros_is_up, main_window.ross_loss_signal, Hz=1)
    ros_state_checker.start()
    # Needed for ctrl+c to work on pyqt applications
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    sys.exit(app.exec_())
