import os
from PyQt5 import QtCore, QtGui, QtWidgets, uic
from PyQt5.QtCore import QUrl
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtWebKitWidgets import QWebView
import rospy


class RRQWebView(QWebView):
    def __init__(self, parent=None):
        super(self.__class__, self).__init__(parent)
        self.setWindowTitle('Loading Web UI...')
        self.titleChanged.connect(self.adjustTitle)
        self.load(rospy.get_param(
            "~web_ui_url", "https://www.robosynthesis.com/"))

    def load(self, url):
        self.setUrl(QUrl(url))

    def adjustTitle(self):
        self.setWindowTitle(self.title())


# class OverlayedWidget(QStackedWidget):
