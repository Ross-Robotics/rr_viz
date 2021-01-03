import os
from PyQt5 import QtCore, QtGui, QtWidgets, uic
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import pyqtSlot
import rospy
from rviz_minimap import RVizMinimap
from rviz_dev import RVizDev
from rviz_sphere import RVizSphere
import rviz


class ROSServiceMonitor(object):
''' Encapsulates the logic that the widget is disabled with the service is not pingable. Code duplication with ServiceAction'''

    def __init__(self, parent=None, srvName="", srvType=None ,cb=None):
        super(self.__class__, self).__init__(parent)
        self._srvConnected = False
        self.isSetup = False    
        if srvName:                              
            self.setup(srvName,srvType,cb)
        

    def setup(self, srvName, srvType,cb=None):
        """[Sets up the timer and service client ]

        Args:
            srvName ([type]): [service name]
            srvType ([type]): [service type]
            cb (function) (optional): [ arg boolean representing new state (connectec/not connected)]
        """        
        self.srvName = srvName
        self.srvType = srvType
        self.cb = cb
        self._srv_client = rospy.ServiceProxy(
            self.srvName, self.srvType)
        self.srv_timer = rospy.Timer(rospy.Duration(
            5), lambda event: self.ping_service())
        self.isSetup = True
       
     def ping_srv(self):
         """[Pings the service and tracks the connected state based on pinagbility (if it times out - not connected). Calls callback with true/false if its connected/not connected]
         """         
        if rospy.is_shutdown():
            return 
        try:
            self._srv_client.wait_for_service(timeout=rospy.Duration(3.0))
            if not self._srv_connected:
                if self.cb:
                    self.cb(True)
                rospy.loginfo('Connected to {}.'.format(self.srv_name))                
            self._srv_connected = True
        except:
            rospy.loginfo_throttle(30,
                                   "Lost Connection to {}".format(self.srv_name) if self._srv_connected else "Failed to connect to {}".format(self.srv_name))
            self._srv_connected = False
            if self.cb:
                    self.cb(False)

    def is_connected(self):
        return self._srv_connected

