#!/usr/bin/env python
import os
import rospy
from PyQt5 import QtGui, QtWidgets, uic, QtCore
from PyQt5.QtWidgets import QApplication, QWidget, QInputDialog, QLineEdit, QFileDialog, QMessageBox, QListWidgetItem
from std_srvs.srv import Trigger

current_dir = os.path.dirname(os.path.abspath(__file__))
rr_viz_dir = _dir = os.path.dirname(os.path.dirname(current_dir))
Form, Base = uic.loadUiType(os.path.join(current_dir, "docking_panel.ui"))


class DockingPanelWidget(Base, Form):
    ''' The only reason this exists so that we can connect mission_editor to command_panel  easily '''
    def __init__(self, parent=None):
        super(self.__class__, self).__init__(parent)
        self.setupUi(self)
        self.setDockingPoseButton.setup("/envirobot_Interface/save_dock_approach",Trigger)
        self.goToDockButton.setup("/envirobot_Interface/go_to_base",Trigger)        
