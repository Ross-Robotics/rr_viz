import os
import time
from PyQt5 import QtCore, QtGui, QtWidgets, uic
from PyQt5.QtCore import QUrl
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtWebKitWidgets import QWebView
import rospy
#check https://stackoverflow.com/questions/11060439/filling-out-a-form-using-pyqt-and-qwebview
#document.getElementById("search-input").value='test2'
#removing the top bar:
#document.getElementsByClassName('nav')[0].parentNode.removeChild(document.getElementsByClassName('nav')[0])
#removes the grey line with date/time
#document.getElementsByClassName('status-wrapper StatusBarGeneral')[0].parentNode.removeChild(document.getElementsByClassName('status-wrapper StatusBarGeneral')[0])
#clicks the login button
#document.getElementById('login-button').click()
# fills the password
#document.getElementById('mat-input-1').value = 'symetricas'
#fuills username
# document.getElementById('mat-input-0').value = 'advanced'
#http://10.42.0.59/ui/en/hosts/sn23n-180004/remote-control

#work plan:
# load the intended URL
# check if we got to the correct URL or if we are in the login screen
# if we are in the login screen input username and password
#if we are in the indeded screen lets remove unwanted elements
# profit

class RRQWebViewTab(QWebView):
    def __init__(self, parent=None):
        super(self.__class__, self).__init__(parent)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._after_loading_timer)
        self.timer.setSingleShot(True)
        self.setWindowTitle('Loading Web UI...')
        self.titleChanged.connect(self.adjustTitle)
        self.load(rospy.get_param(
            "~web_ui_url_tab", "https://www.bbc.co.uk/"))
        self.loadFinished.connect(self._on_load_finished)



    def load(self, url):
        self.setUrl(QUrl(url))


    def adjustTitle(self):
        self.setWindowTitle(self.title())

    def _on_load_finished(self):
        print("FINISHED LOADING")
        print(self.url().toString())
        self.timer.start(3000)


    def _after_loading_timer(self):
        print(self.url().toString())
        frame = self.page().currentFrame()
        frame.evaluateJavaScript("document.getElementById('mat-input-0').value = 'advanced'")
        frame.evaluateJavaScript("document.getElementById('mat-input-1').value = 'symetricas'")
        frame.evaluateJavaScript("document.getElementById('login-button').click()")


# class OverlayedWidget(QStackedWidget):
