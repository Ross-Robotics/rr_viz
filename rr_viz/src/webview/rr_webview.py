import os
from PyQt5 import QtCore, QtGui, QtWidgets, uic
from PyQt5.QtCore import QUrl
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtWebKitWidgets import QWebView
from PyQt5.QtWebKitWidgets import QWebPage
import rospy
from enum import Enum


class RRQWebView(QWebView):

    def __init__(self, parent=None):
        super(self.__class__, self).__init__(parent)
        self.loading_stages = Enum('Loading_stages', 'CONNECTING AUTENTICATING FORMATTING FINISHED')
        self.loading_stage = self.loading_stages.CONNECTING
        self.timer_period = 500
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._after_loading_timer)
        self.timer.setSingleShot(True)
        self.titleChanged.connect(self.adjustTitle)
        self.load(rospy.get_param(
            "~web_ui_url", "https://www.robosynthesis.com/"))
        self.loadFinished.connect(self._on_load_finished)
        print("Remote Screen ",self.loading_stage)

    class WebPage(QWebPage):
        def javaScriptConsoleMessage(self, msg, line, source):
            print '%s line %d: %s' % (source, line, msg)

    def load(self, url):
        self.setUrl(QUrl(url))

    def adjustTitle(self):
        self.setWindowTitle(self.title())

    def _on_load_finished(self):
        self.loading_stage = self.loading_stages.AUTENTICATING
        print("Remote Screen ",self.loading_stage)
        self.timer.start(self.timer_period)

    def _after_loading_timer(self):
        print("Remote Screen ",self.loading_stage)
        if (self._is_login_page()):
            self._login()
            self.timer.start(self.timer_period)
        else:
            self._remove_gui_elements()

    def _is_login_page(self):
        frame = self.page().currentFrame()
        return bool(frame.evaluateJavaScript("document.getElementsByClassName('login').length"))

    def _login(self):
        frame = self.page().currentFrame()
        frame.evaluateJavaScript("document.getElementById('mat-input-0').value = 'advanced'")
        frame.evaluateJavaScript("document.getElementById('mat-input-1').value = 'symetricas'")
        frame.evaluateJavaScript("document.getElementById('login-button').click()")
        self.loading_stage = self.loading_stages.FORMATTING

    def _remove_gui_elements(self):
        frame = self.page().currentFrame()
        if self._is_remote_screen_loaded():
            frame.evaluateJavaScript("document.getElementsByClassName('nav')[0].parentNode.removeChild(document.getElementsByClassName('nav')[0])")
            frame.evaluateJavaScript("document.getElementsByClassName('status-wrapper StatusBarGeneral')[0].parentNode.removeChild(document.getElementsByClassName('status-wrapper StatusBarGeneral')[0])")
            frame.evaluateJavaScript("document.getElementsByClassName('doseinfo')[0].parentNode.removeChild(document.getElementsByClassName('doseinfo')[0])")
            frame.evaluateJavaScript("document.getElementsByTagName('h1')[0].style.display = 'none'")
            frame.evaluateJavaScript("document.getElementsByTagName('p')[0].style.display = 'none'")
            self.loading_stage = self.loading_stages.FINISHED
            return True
        else:
            self.timer.start(self.timer_period)
            return False

    def _is_remote_screen_loaded(self):
        frame = self.page().currentFrame()
        if frame.evaluateJavaScript("document.getElementsByClassName('nav').length"):
            if frame.evaluateJavaScript("document.getElementsByClassName('status-wrapper StatusBarGeneral').length"):
                if frame.evaluateJavaScript("document.getElementsByClassName('doseinfo').length"):
                    if frame.evaluateJavaScript("document.getElementsByTagName('h1').length"):
                        if frame.evaluateJavaScript("document.getElementsByTagName('p').length"):
                            return True
        return False


#document.getElementsByTagName('body')[0].appendChild(img)
