import os
import sys
import ctypes
import platform
from cefpython3 import cefpython as cef
from PyQt5 import QtCore, QtGui, QtWidgets, uic
from PyQt5.QtCore import QUrl
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtWebKitWidgets import QWebView
from PyQt5.QtWebKitWidgets import QWebPage
import rospy
import rospkg



#http://devtools.fg.oisin.rc-harwell.ac.uk/nightly/7.0/ccp4-src-7.0/checkout/PyQt-x11-gpl-4.11.2/doc/html/qwebview.html

class RRQWebViewTab(QWidget):
    def __init__(self, parent):
        super(QWidget, self).__init__(parent)
        rospack = rospkg.RosPack()
        main_layout = QHBoxLayout(self)
        self.loading_layout = QVBoxLayout(self)
        self.projectS_logo = rospack.get_path('rr_viz') + "/res/projectS_logo.png"
        pixmap = QPixmap(self.projectS_logo)
        self.status_label = QLabel('Connecting to remote screen')
        self.status_label.setFont(QFont('Ubuntu',20,QFont.Bold))
        self.status_label.setHidden(True)
        self.status_label.setAlignment(Qt.AlignCenter)

        #CEF stuff
        sys.excepthook = cef.ExceptHook  # To shutdown all CEF processes on error
        # app = self.CefApplication(sys.argv)
        self.cef_widget = self.CefWidget()

        self.web_gui = self.RRQWebTab()

        self.logo_label = QLabel(self)
        self.logo_label.setPixmap(pixmap)
        self.logo_label.setAlignment(Qt.AlignCenter)
        self.logo_label.setHidden(True)

        self.loading_layout.addWidget(self.logo_label,1)
        self.loading_layout.addWidget(self.status_label,2)
        main_layout.addLayout(self.loading_layout)
        #self.web_gui = self.RRQWebTab()
        #main_layout.addWidget(self.web_gui,1)
        #self.container = QWidget.createWindowContainer(self.cef_widget.hidden_window, parent=self)
        #main_layout.addWidget(self.cef_widget,2)

        self.tab_timer_period = 500
        self.tab_timer = QTimer(self)
        self.tab_timer.timeout.connect(self._visibility_manager)
        self.setLayout(main_layout)
        self.cef_widget.embedBrowser()
        self.cef_container = QWidget.createWindowContainer(self.cef_widget.hidden_window, parent=self)
        main_layout.addWidget(self.cef_container,2)
        self.tab_timer.start(self.tab_timer_period)


    def _visibility_manager(self):
        # self.status_label.setHidden(not self.web_gui.is_hidden)
        # self.logo_label.setHidden(not self.web_gui.is_hidden)
        # self.loading_layout.setEnabled(self.web_gui.is_hidden)
        # self.status_label.setText(self.web_gui.gui_state + " ...")
        self.cef_widget.resizeWindow(self.cef_container.width() ,self.cef_container.height())



    class RRQWebTab(QWebView):
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
                "home_page":"/ui/en/hosts/" + self.gui_serial_number,
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
            elif self.current_url.find(self.gui_url_dict.get("home_page")) >= 0:
                if self.is_finished_loading:
                    self.gui_state = "Remote screen fully loaded"
                    #self.is_hidden = False
                    self.gui_state = "Opening remote screen"
            if self.url_loading_state == "Failed":
                self._load( self.current_url)


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

        class WebPage(QWebPage):
            def javaScriptConsoleMessage(self, msg, line, source):
                print("JS CONSOLE OUTPUT")
                print '%s line %d: %s' % (source, line, msg)


    class CefWidget(QWidget):
        def __init__(self, parent=None):
            # noinspection PyArgumentList
            super(self.__class__, self).__init__(parent)
            cef.Initialize()
            self.parent = parent
            self.hidden_window = None  # Required for PyQt5 on Linux
            self.window_info = cef.WindowInfo()
            self.cef_timer_period = 10
            self.cef_timer = QTimer(self)
            self.cef_timer.timeout.connect(self._cef_on_timer)
            self.cef_timer.start(self.cef_timer_period)

        def _cef_on_timer(self):
            cef.MessageLoopWork()

        def embedBrowser(self):
            self.hidden_window = QWindow()
            self.window_info.SetAsChild(int(self.hidden_window.winId()))
            self.browser = cef.CreateBrowserSync(self.window_info, url="http://10.42.0.59")

        def resizeWindow(self,width,height):
            self.browser.SetBounds(0,0,width,height)

        def resizeEvent(self, event):
            size = event.size()
            self.browser.SetBounds(self.x, self.y, size.width(), size.height())
            self.browser.NotifyMoveOrResizeStarted()
