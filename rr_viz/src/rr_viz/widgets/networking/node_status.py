import os
import rospy
import rospkg
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from functools import partial
from PyQt5 import QtGui, QtWidgets, uic, QtCore
from PyQt5.QtWidgets import QApplication, QWidget, QInputDialog, QLineEdit, QFileDialog, QMessageBox, QListWidgetItem

current_dir = os.path.dirname(os.path.abspath(__file__))
Form, Base = uic.loadUiType(os.path.join(current_dir, "node_status.ui"))


class NodeStatusWidget(Base, Form):

    def __init__(self, name, parent=None):
        super(self.__class__, self).__init__(parent)
        self.setupUi(self)
        # print("NodeStatusWidget created")
        self.nodeNameLbl.setText(name)
