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
Form, Base = uic.loadUiType(os.path.join(current_dir, "status_mission.ui"))


class StatusMissionWidget(Base, Form):
    set_mission_status_label_signal = QtCore.pyqtSignal(str)      

    def __init__(self, parent=None):
        super(self.__class__, self).__init__(parent)
        self.setupUi(self)
        print("StatusMissionWidget Loaded")

        self.set_label = self.set_mission_status_label_signal
        self.set_label.connect(self.set_mission_status_label_slot)

        self.sub=rospy.Subscriber("status_test", String, lambda data: self.set_label.emit(data.data))
        
        

    def set_mission_status_label_slot(self,data):
        self.missionStatusLbl.setText(data)
