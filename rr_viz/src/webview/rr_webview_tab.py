#!/usr/bin/env python
import sys
from cefpython3 import cefpython as cef
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
import rospy
import managers.file_management as file_management

class RRQWebViewTab(QWidget):
    def __init__(self, parent):
        super(QWidget, self).__init__(parent)
        main_layout = QHBoxLayout(self)
        self.loading_layout = QVBoxLayout(self)
        self.projectS_logo = file_management.get_rrviz_resdir()+ "/projectS_logo.png"
        pixmap = QPixmap(self.projectS_logo)
        self.status_label = QLabel('Connecting to VeriFinder')
        self.status_label.setFont(QFont('Ubuntu',20,QFont.Bold))
        self.status_label.setHidden(True)
        self.status_label.setAlignment(Qt.AlignCenter)

        #CEF stuff
        sys.excepthook = cef.ExceptHook  # To shutdown all CEF processes on error
        self.cef_widget = self.CefWidget()

        self.logo_label = QLabel(self)
        self.logo_label.setPixmap(pixmap)
        self.logo_label.setAlignment(Qt.AlignCenter)
        self.logo_label.setHidden(True)

        self.loading_layout.addWidget(self.logo_label,1)
        self.loading_layout.addWidget(self.status_label,2)
        main_layout.addLayout(self.loading_layout)

        self.tab_timer_period = 500
        self.tab_timer = QTimer(self)
        self.tab_timer.timeout.connect(self._window_manager)
        self.setLayout(main_layout)
        self.cef_widget.embedBrowser()
        self.cef_container = QWidget.createWindowContainer(self.cef_widget.hidden_window, parent=self)
        main_layout.addWidget(self.cef_container,2)
        self.tab_timer.start(self.tab_timer_period)


    def _window_manager(self):
        self.cef_widget.resizeWindow(self.cef_container.width() ,self.cef_container.height())
        if self.cef_widget.browser.GetUrl() == '':
            self.cef_container.setHidden(True)
            self.logo_label.setHidden(False)
            self.status_label.setHidden(False)
        else:
            self.cef_container.setHidden(False)
            self.logo_label.setHidden(True)
            self.status_label.setHidden(True)



    class CefWidget(QWidget):
        def __init__(self, parent=None):
            # noinspection PyArgumentList
            super(self.__class__, self).__init__(parent)
            self.gui_hostname = rospy.get_param("~gui_hostname", "10.42.0.1")
            self.gui_url_dict = {
                "host_page":"/ui/en/",
                "host_login_transition":"/ui/en/login",
                "login_page":"/ui/en/login?redirectUrl=",
                "login_landing_transition":"/ui/en/landing/true",
                "home_page":"/ui/en/hosts/" + self.gui_hostname,
                "remote_screen":"/ui/en/hosts/" + self.gui_hostname + "/remote-control"
            }
            cef.Initialize()
            self.parent = parent
            self.hidden_window = None  # Required for PyQt5 on Linux
            self.browser = cef.PyBrowser()
            self.window_info = cef.WindowInfo()
            self.cef_timer_period = 10
            self.cef_timer = QTimer(self)
            self.cef_timer.timeout.connect(self._cef_on_timer)
            self.cef_timer.start(self.cef_timer_period)


        def _url_builder(self,hostname,page):
            url = "http://" + hostname + page
            return url

        def _cef_on_timer(self):
            cef.MessageLoopWork()
            if(self.browser.GetUrl() == self._url_builder(self.gui_hostname,self.gui_url_dict.get("login_page"))):
                self._login()
            if self.browser.GetUrl() == '':
                self.browser.LoadUrl(self._url_builder(self.gui_hostname,self.gui_url_dict.get("host_page")))

        def _login(self):
            self.browser.ExecuteJavascript("document.getElementById('mat-input-0').value = 'advanced'")
            self.browser.ExecuteJavascript("document.getElementById('mat-input-1').value = 'symetricas'")
            self.browser.ExecuteJavascript("document.getElementById('login-button').click()")

        def _browser_loading(self):
            print("BROWSER LOADING...")

        def embedBrowser(self):
            self.hidden_window = QWindow()
            self.window_info.SetAsChild(int(self.hidden_window.winId()))
            self.browser = cef.CreateBrowserSync(self.window_info, url="http://10.42.0.59")

        def resizeWindow(self,width,height):
            self.browser.SetBounds(0,0,width,height)
            #print(self.browser.IsLoading())
