import os
from PyQt5 import QtCore, QtGui, QtWidgets, uic
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import pyqtSlot


# import rviz
from rviz_frame import RViz_frame
import managers.file_management as file_management

# Notes:
# This is a wrapper for rviz_fram that just loads in the right config

current_dir = os.path.dirname(os.path.abspath(__file__))
Form, Base = uic.loadUiType(os.path.join(current_dir, "blank.ui"))
# NOTE: Change this to change what frame config is loaded by this view
RIVZ_CONFIG = "rviz_textured_sphere.rviz"


class RVizSphere(Base, Form, RViz_frame):
    def __init__(self, parent=None):
        super(self.__class__, self).__init__(parent)
        RViz_frame.__init__(self)
        # super(RViz_frame.__class__, self).__init__()
        self.setupUi(self)
        print("RvizView created")
        self.set_rviz_config(RIVZ_CONFIG)
        # file_management.get_splash()
        self.load_rviz_frame(hide_menu=True, hide_status=True, splash="")
