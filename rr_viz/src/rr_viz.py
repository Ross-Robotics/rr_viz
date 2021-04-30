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
    # ross_loss_signal = QtCore.pyqtSignal(bool)

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

        # self.ross_loss_signal.connect(self.ros_loss_slot)
        self.ros_loss_triggered = False
        self.fp = FastPing()
        self.rosMasterIP = '10.42.0.1'
        self.fp_status = "alive"          
        
        self.timer_period = 1000
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.ros_is_up)
        self.timer.start(self.timer_period)

    def ros_is_up(self):
        result = self.fp.ping(targets=[self.rosMasterIP])
        ros_master_hostname = ''.join(result.keys())
        if not self.fp_status in result[ros_master_hostname] and not self.ros_loss_triggered:
            rospy.logerr("Connection to ROS MASTER has been lost")
            self.ros_loss_triggered = True
            # msg = QMessageBox()
            # msg.setIcon(QMessageBox.Warning)
            # msg.setText(
            #     "RR_VIZ lost connection to the ROS MASTER. Please wait")
            # msg.setWindowTitle("Connection Lost")
            # exit_rr_viz = msg.addButton('Exit RR_VIZ', QMessageBox.AcceptRole)
            # msg.exec_()
            # if msg.clickedButton() == exit_rr_viz:
            #     sys.exit()            
        elif self.fp_status in result[ros_master_hostname] and self.ros_loss_triggered:
            rospy.loginfo("Connection to ROS MASTER has been restored")
            self.ros_loss_triggered = False
            

    # def ros_loss_slot(self, flag):
    #     msg = QMessageBox()
    #     print("flag: {}".format(flag))
    #     # print("trig: {}".format(self.ros_loss_triggered))
    #     if not flag and not self.ros_loss_triggered:
    #         # print("no ros")
    #         self.ros_loss_triggered = True

    #         msg.setIcon(QMessageBox.Warning)
    #         msg.setText(
    #             "RR_VIZ lost connection to the ROS MASTER. Please wait")
    #         msg.setWindowTitle("Connection Lost")
    #         exit_rr_viz = msg.addButton('Exit RR_VIZ', QMessageBox.AcceptRole)
    #         continue_rr_viz = msg.addButton('Continue', QMessageBox.AcceptRole)
    #         msg.exec_()
    #         if msg.clickedButton() == exit_rr_viz:
    #             sys.exit()
    #     elif flag and self.ros_loss_triggered:
    #         self.ros_loss_triggered = False
    #         # print("BACK")
    #         msg.setText("Connection to ROS MASTER is re-established. Click Continue to proceed")
    #         msg.exec_()


if __name__ == '__main__':
    rospy.init_node("rr_viz")
    app = QApplication(sys.argv)
    main_window = RossRoboticsRViz()
    main_window.show()
    # ros_state_checker = rr_qt_helper.StateCheckerTimer(
    #     main_window.ros_is_up, main_window.ross_loss_signal, Hz=1)
    # ros_state_checker.start()
    # Needed for ctrl+c to work on pyqt applications
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    sys.exit(app.exec_())
