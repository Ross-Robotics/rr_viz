import os
from PyQt5 import QtCore, QtGui, QtWidgets, uic
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import pyqtSlot
import rospy
from std_msgs.msg import String

class ROSTopicLabel(QtWidgets.QLabel):
    """[Label that displayes ROS String Topic]
    """

    def __init__(self, parent=None, topic=""):
        super(self.__class__, self).__init__(parent)
        self.isSetup = False
        if topic:
            self.setup(topic)

    def setup(self, topic):
        self.topic = topic
        self.sub = rospy.Subscriber(self.topic, String, lambda msg : self.setText(msg.data))
        self.isSetup = True
