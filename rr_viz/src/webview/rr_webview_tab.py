import os
import time
from PyQt5 import QtCore, QtGui, QtWidgets, uic
from PyQt5.QtCore import QUrl
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtWebKitWidgets import QWebView
import rospy


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
        self.timer.start(3000)


    def _after_loading_timer(self):
        print("Logging into the Verifinder ... ")
        if (self._is_login_page()):
            self._login()


    def _is_login_page(self):
        frame = self.page().currentFrame()
        return bool(frame.evaluateJavaScript("document.getElementsByClassName('login').length"))

    def _login(self):
        frame = self.page().currentFrame()
        frame.evaluateJavaScript("document.getElementById('mat-input-0').value = 'advanced'")
        frame.evaluateJavaScript("document.getElementById('mat-input-1').value = 'symetricas'")
        frame.evaluateJavaScript("document.getElementById('login-button').click()")

    def _remove_gui_elements(self):
        frame = self.page().currentFrame()
        pass
