import os
from PyQt5 import QtCore, QtGui, QtWidgets, uic
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import pyqtSlot
import rospy
from ros_service_monitor import ROSServiceMonitor
from std_srvs.srv import Trigger



class ROSServiceClientQButton(QtWidgets.QPushButton):
    def __init__(self, parent=None, srvName="", srvType=None, createRequestCb=None):
        """[QButton that calls a ros service:
        is enabled/disabled based on ping to service
        calls service upon click.
        service request empty or custom]

        Args:
            parent ([type], optional): [description]. Defaults to None.
            srvName (str, optional): [service name]. Defaults to "".
            srvType (class, optional): [service type]. Defaults to None.
            createRequestCb ([type], optional): [description]. Defaults to None.
        """        
        super(self.__class__, self).__init__(parent)
        self.isSetup = False
        if srvName:
            self.setup(srvName, srvType, createRequestCb)

    def setup(self, srvName, srvType, createRequestCb=None):
        """[summary]

        Args:
            srvName (str, optional): [service name].
            srvType (class, optional): [service type]. 
            createRequestCb ([function], optional): [description]. Defaults to empty creation based on type.
        """        
        self.srvName = srvName
        self.srvType = srvType
        if createRequestCb:
            self.createRequestCb = createRequestCb
        else:
            self.createRequestCb = self.createEmptyRequest
        
        # Enabled depends if the service is visible
        self.stateMonitor = ROSServiceMonitor(None, self.srvName, self.srvType, self.serviceStateCb)
        self._srv_client = rospy.ServiceProxy(
            self.srvName, self.srvType)
        self.clicked.connect(self.call_service)
        self.isSetup = True

    def serviceStateCb(self, state):
        """[executes when service changes state (connected/not)]
        """
        # This will likely not work because  it needs a signal will check
        self.setEnabled(state)

    def createEmptyRequest(self):
        """[Creates an empty request message with no arguments based on srvType]

        Returns:
            [class]: [srvTypeRequest]
        """
        return self.srvType._request_class()

    def call_service(self):        
        response=self._srv_client.call(self.createRequestCb())
