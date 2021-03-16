import os
from PyQt5 import QtCore, QtGui, QtWidgets, uic
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import pyqtSlot
import rospy


# import rviz
from rviz_frame import RViz_frame
import managers.file_management as file_management

# Notes:
# This is a wrapper76 for rviz_fram that just loads in the right config

current_dir = os.path.dirname(os.path.abspath(__file__))
Form, Base = uic.loadUiType(os.path.join(current_dir, "blank.ui"))
# NOTE: Change this to change what frame config is loaded by this view
RIVZ_CONFIG = "minimap.rviz"


class RVizMinimap(Base, Form, RViz_frame):
    def __init__(self, parent=None):
        super(self.__class__, self).__init__(parent)
        RViz_frame.__init__(self)
        # super(RViz_frame.__class__, self).__init__()
        self.setupUi(self)
        print("RVizMinimap created")
        self.set_rviz_config(RIVZ_CONFIG)
        # file_management.get_splash()
        self.load_rviz_frame(hide_menu=True, hide_status=True, splash="")
        self.poseEstimates = []
        self.timer = QTimer(self)
        for button in self.findChildren(QtWidgets.QToolButton):
            if "Estimate" in button.text():
                self.poseEstimates.append(button)

        # Workaround for the refresh that rviz does
        self.timer.timeout.connect(self.set_text_slot)
        self.timer.start(100)

    def set_text_slot(self):
        try:
            self.poseEstimates[2].setText("Set Pose Estimate")
            self.poseEstimates[3].setText("Spawn Waypoint")
        except:
            pass
