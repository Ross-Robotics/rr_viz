#!/usr/bin/env python
from PyQt5.QtCore import QUrl
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtWebKitWidgets import QWebView
from PyQt5.QtWebKitWidgets import QWebPage
import rospy
import managers.file_management as file_management

class RRQWebView(QWidget):
    def __init__(self, parent):
        super(QWidget, self).__init__(parent)
        main_layout = QHBoxLayout(self)
        self.loading_layout = QVBoxLayout(self)
        self.setMaximumWidth(400)
        self.projectS_logo = file_management.get_rrviz_resdir()+ "/projectS_logo.png"
        pixmap = QPixmap(self.projectS_logo)
        self.status_label = QLabel('Connecting to remote screen')
        self.status_label.setFont(QFont('Ubuntu',20,QFont.Bold))
        self.status_label.setHidden(True)
        self.status_label.setAlignment(Qt.AlignCenter)

        self.logo_label = QLabel(self)
        self.logo_label.setPixmap(pixmap)
        self.logo_label.setAlignment(Qt.AlignCenter)
        self.logo_label.setHidden(True)

        self.loading_layout.addWidget(self.logo_label)
        self.loading_layout.addWidget(self.status_label)
        main_layout.addLayout(self.loading_layout,1)
        self.remote_screen = self.RRQWebGui()
        main_layout.addWidget(self.remote_screen )
        self.timer_period = 500
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._visibility_manager)
        self.setLayout(main_layout)
        self.timer.start(self.timer_period)

    def _visibility_manager(self):
        self.status_label.setHidden(not self.remote_screen.is_hidden)
        self.logo_label.setHidden(not self.remote_screen.is_hidden)
        self.loading_layout.setEnabled(self.remote_screen.is_hidden)
        self.status_label.setText(self.remote_screen.gui_state + " ...")

    class RRQWebGui(QWebView):
        def __init__(self, parent=None):
            super(self.__class__, self).__init__(parent)
            # gui variables
            self.gui_hostname = rospy.get_param("~gui_hostname", "10.42.0.1")
            self.gui_serial_number = rospy.get_param("~gui_serial_number", "sn23n-180004")
            self.url_loading_state = "Not started"
            self.gui_state = "Connecting"
            self.current_url_loading_progress = 0
            self.is_finished_loading = False
            self.is_hidden = True
            self.gui_available = False
            self.current_url = "None"
            self.current_page = "None"
            self.gui_url_dict = {
                "host_page":"/ui/en/",
                "host_login_transition":"/ui/en/login",
                "login_page":"/ui/en/login?redirectUrl=",
                "login_landing_transition":"/ui/en/landing/true",
                "home_page":"/ui/en/hosts/",
                "remote_screen":"/ui/en/hosts/" + self.gui_serial_number + "/remote-control"
            }
            # timers config
            self.timer_period = 500
            self.timer = QTimer(self)
            self.timer.timeout.connect(self._gui_manager)
            # load signal connection
            self.loadFinished.connect(self._on_load_finished)
            self.loadStarted.connect(self._on_load_started)
            self.loadProgress.connect(self._on_load_progress)
            self.urlChanged.connect(self._on_url_change)
            self.setHidden(self.is_hidden)
            self.timer.start(self.timer_period)

        def _gui_manager(self):
            self.setHidden(self.is_hidden)
            if self.current_url == "None":
                self.load_page("host_page")
            elif self.current_url.find(self.gui_url_dict.get("login_page")) >= 0:
                self.gui_state = "Logging into the GUI"
                self._login()
            elif self.current_url.find(self.gui_url_dict.get("remote_screen")) >= 0:
                if self.is_finished_loading:
                    if self._remove_control_gui_elements() or self.gui_state == "Remote screen fully loaded":
                        self.gui_state = "Remote screen fully loaded"
                        self.is_hidden = False
                    else:
                        self.gui_state = "Configuring GUI"
            elif self.current_url.find(self.gui_url_dict.get("home_page")) >= 0:
                if self.is_finished_loading:
                    self.gui_state = "Opening remote screen"
                    self.find_serial_number(self.current_url,'/')
                    self.gui_url_dict.update(remote_screen = "/ui/en/hosts/" + self.gui_serial_number + "/remote-control")
                    self.load_page("remote_screen")
            if self.url_loading_state == "Failed":
                self._load( self.current_url)

        def find_serial_number(self,s,ch):
            x = [i for i, ltr in enumerate(s) if ltr == ch]
            final = x[len(x)-1]
            self.gui_serial_number = s[final+1:]

        def load_page(self,page):
            page_tail = self.gui_url_dict.get(page)
            if page_tail != None:
                self.is_finished_loading = False
                url = self._url_builder(self.gui_hostname,page_tail)
                self._load(url)
                self.current_url = url
                return True
            return False

        def _url_builder(self,hostname,page):
            url = "http://" + hostname + page
            return url

        def _load(self, url):
            print("Loading -> " + url)
            self.url_loading_state = "Not started"
            self.setUrl(QUrl(url))
            self.is_finished_loading = False

        def _on_load_finished(self,sucess):
            print("Finished Loading -> " + self.url().toString())
            if sucess:
                self.url_loading_state = "Finished"
            else:
                self.url_loading_state = "Failed"
            self.is_finished_loading = True

        def _on_load_started(self):
            print("Started Loading -> " + self.url().toString())
            self.url_loading_state = "Started"
            self.is_finished_loading = False

        def _on_load_progress(self,load):
            self.current_url_loading_progress = load
            print("URL loaded: " + str(load) + "%")
            pass

        def _on_url_change(self):
            print("Web page URL changed to:")
            self.current_url = self.url().toString()
            print(self.current_url)

        def _login(self):
            print("Logging into the GUI")
            frame = self.page().currentFrame()
            frame.evaluateJavaScript("document.getElementById('mat-input-0').value = 'advanced'")
            frame.evaluateJavaScript("document.getElementById('mat-input-1').value = 'symetricas'")
            frame.evaluateJavaScript("document.getElementById('login-button').click()")

        def _remove_control_gui_elements(self):
            frame = self.page().currentFrame()
            if self._is_remote_screen_loaded():
                frame.evaluateJavaScript("document.getElementsByClassName('nav')[0].parentNode.removeChild(document.getElementsByClassName('nav')[0])")
                frame.evaluateJavaScript("document.getElementsByClassName('status-bar')[0].parentNode.removeChild(document.getElementsByClassName('status-bar')[0])")
                frame.evaluateJavaScript("document.getElementsByClassName('doseinfo')[0].parentNode.removeChild(document.getElementsByClassName('doseinfo')[0])")
                frame.evaluateJavaScript("document.getElementsByTagName('h1')[0].style.display = 'none'")
                frame.evaluateJavaScript("document.getElementsByTagName('p')[0].style.display = 'none'")
                return True
            else:
                return False

        def _is_remote_screen_loaded(self):
            frame = self.page().currentFrame()
            if frame.evaluateJavaScript("document.getElementsByClassName('nav').length"):
                if frame.evaluateJavaScript("document.getElementsByClassName('status-bar').length"):
                    if frame.evaluateJavaScript("document.getElementsByClassName('doseinfo').length"):
                        if frame.evaluateJavaScript("document.getElementsByTagName('h1').length"):
                            if frame.evaluateJavaScript("document.getElementsByTagName('p').length"):
                                return True
            return False
