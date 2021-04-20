import os
from PyQt5 import QtCore, QtGui, QtWidgets, uic
from PyQt5.QtCore import QUrl
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtWebKitWidgets import QWebView
from PyQt5.QtWebKitWidgets import QWebPage
import rospy
import rospkg
from enum import Enum
import urllib

# TODO
# make a timer to start as the widget is init
# load the placeholder html
# check if the webpage is available
#     if available load
#     if need to autenticate
#         autenticate
#     if need to remove elements
#         remove elements
#     if need action
#         do action
#initial url
#PyQt5.QtCore.QUrl(u'http://10.42.0.59/ui/en/')
#transition login url
#but with the hostname
#PyQt5.QtCore.QUrl(u'http://10.42.0.59/ui/en/login')
#login url
#PyQt5.QtCore.QUrl(u'http://10.42.0.59/ui/en/login?redirectUrl=')
#transition url
#PyQt5.QtCore.QUrl(u'http://10.42.0.59/ui/en/landing/true')
#home url
#PyQt5.QtCore.QUrl(u'http://10.42.0.59/ui/en/hosts/sn23n-180004')
#remote control url
#PyQt5.QtCore.QUrl(u'http://10.42.0.59/ui/en/hosts/sn23n-180004/remote-control')
#http://devtools.fg.oisin.rc-harwell.ac.uk/nightly/7.0/ccp4-src-7.0/checkout/PyQt-x11-gpl-4.11.2/doc/html/qwebview.html


class RRQWebView(QWebView):
    def __init__(self, parent=None):
        super(self.__class__, self).__init__(parent)
        rospack = rospkg.RosPack()
        self.placeholder_webpage_path = "file://" + rospack.get_path('rr_viz')+"/res/placeholder_webpage.html"
        self.gui_hostname = rospy.get_param("~gui_hostname", "https://www.robosynthesis.com/")
        self.loading_stages = Enum('Loading_stages', 'NOT_AVAILABLE CONNECTING AUTENTICATING TRANSITION FORMATTING FINISHED')
        self.url_loading_state = "Not started"
        self.loading_stage = self.loading_stages.CONNECTING
        self.gui_available = False
        self.gui_placeholder_loaded = False
        self.timer_period = 500
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._gui_manager)
        self.loadFinished.connect(self._on_load_finished)
        self.loadStarted.connect(self._on_load_started)
        self.loadProgress.connect(self._on_load_progress)
        self.urlChanged.connect(self._on_url_change)
        #self.load(self.placeholder_webpage_path)
        #self.load(self.gui_url)
        self.loading_stage = self.loading_stages.CONNECTING
        print("Remote Screen ",self.loading_stage)
        self.timer.start(self.timer_period)

    def _gui_manager(self):
        print("Remote Screen ",self.loading_stage)
        if self.gui_available:
            pass
        elif not self.gui_placeholder_loaded:
            if self.url_loading_state == "Not started":
                self.load(self.placeholder_webpage_path)
                print("Loading placeholder")
            elif self.url_loading_state == "Finished":
                self._show_overlay_text("Connecting ...")
    def _is_the_website_up(self, url):
        code = 0
        try:
            code = urllib.urlopen(self.url).getcode()
            print(code)
            if code == 200 or code == 401:
                return True
        except:
            print("Website is not up")
            return False

    def load(self, url):
        print("Loading -> " + self.url().toString())
        print(url)
        self.setUrl(QUrl(url))

    def _on_load_finished(self):
        self.url_loading_state = "Finished"

    def _on_load_started(self):
        print("WEB PAGE LOADING SIGNAL")
        self.url_loading_state = "Started"
        #self._show_connecting_image()

    def _on_load_progress(self,load):
        self.url_loading_state = self.url().toString() + " Loading -> " + str(load) + "%"

    def _on_url_change(self):
        print("Web page URL changed to:")
        print(self.url().host())
        print(self.url().toString())
        #self._show_connecting_image()

    def _is_login_page(self):
        frame = self.page().currentFrame()
        #self._show_connecting_image()
        return bool(frame.evaluateJavaScript("document.getElementsByClassName('login').length"))

    def _login(self):
        frame = self.page().currentFrame()
        frame.evaluateJavaScript("document.getElementById('mat-input-0').value = 'advanced'")
        frame.evaluateJavaScript("document.getElementById('mat-input-1').value = 'symetricas'")
        frame.evaluateJavaScript("document.getElementById('login-button').click()")
        self.loading_stage = self.loading_stages.FORMATTING

    def _remove_control_gui_elements(self):
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
            #self.timer.start(self.timer_period)
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

    def _show_overlay_text(self, display_text = "Loading..."):
        text_cmd = "var text = document.createTextNode('" + display_text + "')"
        frame = self.page().currentFrame()
        frame.evaluateJavaScript("var overlay = document.createElement('overlay')")
        frame.evaluateJavaScript("var att = document.createAttribute('class')")
        frame.evaluateJavaScript("var st = document.createAttribute('style')")
        frame.evaluateJavaScript("att.value = 'overlay'")
        frame.evaluateJavaScript("st.value = 'position: fixed;font-family: Helvetica;font-weight: bold; text-align: center; padding: 50% 0;  display: none;  width: 100%;  height: 100%;  top: 0;  left: 0;  right: 0;  bottom: 0;  background-color: rgba(255,255,255,1.0);  z-index: 2;  cursor:pointer;'")
        frame.evaluateJavaScript(text_cmd)
        frame.evaluateJavaScript("overlay.appendChild(text)")
        frame.evaluateJavaScript("overlay.setAttributeNode(st)")
        frame.evaluateJavaScript("overlay.setAttributeNode(att)")
        frame.evaluateJavaScript("document.getElementsByTagName('body')[0].appendChild(overlay)")
        frame.evaluateJavaScript("document.getElementsByClassName('overlay')[0].style.display = 'block'")
        #var text = document.createTextNode("Loading Verifinder remote screen.");
        #overlay.appendChild(text)

    def _remove_overlay_text(self):
        frame = self.page().currentFrame()
        print("Removing the overlay")
        print(frame.evaluateJavaScript("document.getElementsByClassName('overlay')[0].lemgth"))
        frame.evaluateJavaScript("document.getElementsByClassName('overlay')[0].style.display = 'none'")
