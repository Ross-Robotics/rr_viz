import os
import rospy
import rosnode
import rospkg
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from functools import partial
from PyQt5 import QtGui, QtWidgets, uic, QtCore
from PyQt5.QtWidgets import QApplication, QWidget, QInputDialog, QLineEdit, QFileDialog, QMessageBox, QListWidgetItem

from node_status import NodeStatusWidget

current_dir = os.path.dirname(os.path.abspath(__file__))
from node_status import NodeStatusWidget


class NodeStatusYocsMux(Base, Form, NodeStatusWidget):
    set_label_signal = QtCore.pyqtSignal(int)      
    def __init__(self, parent=None):
        super(self.__class__, self).__init__(parent)
        self.setupUi(self)        