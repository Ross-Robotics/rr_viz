#!/usr/bin/env python
import sys
from subprocess import Popen, PIPE
import signal
import os
import rospy
import rosgraph
import time
import traceback, sys

from PyQt5.QtWidgets import *
from PyQt5 import QtCore, QtGui
from PyQt5.QtCore import *
from PyQt5.QtGui import *

from rr_viz_tabs import RRVizTabs
from helpers import rr_qt_helper
import managers.file_management as file_management

class WorkerSignals(QObject):
    '''
    Defines the signals available from a running worker thread.

    Supported signals are:

    finished
        No data

    error
        tuple (exctype, value, traceback.format_exc() )

    result
        object data returned from processing, anything

    progress
        int indicating % progress

    '''
    finished = pyqtSignal()
    error = pyqtSignal(tuple)
    result = pyqtSignal(object)
    progress = pyqtSignal(int)

class Worker(QRunnable):
    '''
    Worker thread

    Inherits from QRunnable to handler worker thread setup, signals and wrap-up.

    :param callback: The function callback to run on this worker thread. Supplied args and
                     kwargs will be passed through to the runner.
    :type callback: function
    :param args: Arguments to pass to the callback function
    :param kwargs: Keywords to pass to the callback function

    '''

    def __init__(self, fn, *args, **kwargs):
        super(Worker, self).__init__()

        # Store constructor arguments (re-used for processing)
        self.fn = fn
        self.args = args
        self.kwargs = kwargs
        self.signals = WorkerSignals()

        # Add the callback to our kwargs
        self.kwargs['progress_callback'] = self.signals.progress

    @pyqtSlot()
    def run(self):
        '''
        Initialise the runner function with passed args, kwargs.
        '''

        # Retrieve args/kwargs here; and fire processing using them
        try:
            result = self.fn(*self.args, **self.kwargs)
        except:
            traceback.print_exc()
            exctype, value = sys.exc_info()[:2]
            self.signals.error.emit((exctype, value, traceback.format_exc()))
        else:
            self.signals.result.emit(result)  # Return the result of the processing
        finally:
            self.signals.finished.emit()  # Done


class RossRoboticsRViz(QMainWindow):
    def __init__(self, parent=None,*args, **kwargs):
        super(QMainWindow, self).__init__(parent,*args, **kwargs)
        self.setWindowTitle("Ross Robotics RViz")
        logo_path = file_management.get_rrviz_resdir() + "/low_level_gui.png"
        self.setWindowIcon(QtGui.QIcon(logo_path))
        self.x = 10
        self.y = 10
        self.width = 1482
        self.height = 868
        self.setGeometry(self.x, self.y, self.width, self.height)
        file_management.check_user_dir()
        self.create_node()
        self.tabWidget = RRVizTabs(self)
        self.setCentralWidget(self.tabWidget)
        self.threadpool = QThreadPool()
        print(">>>>>Multithreading with maximum %d threads" % self.threadpool.maxThreadCount())

        worker = Worker(self.recurring_process) # Any other args, kwargs are passed to the run function
        worker.signals.result.connect(self.print_output)
        worker.signals.finished.connect(self.thread_complete)
        self.threadpool.start(worker)


    def recurring_process(self,progress_callback):
        while 1:
            alive = self.ping("192.168.10.100",5)
            if not alive:
                print("Robot is not reachable via Network, GUI shutting down")
                os.system('zenity --error --no-wrap  --text="Connection to the EXTRM lost. Closing GUI."')
                os.kill(pid, signal.SIGTERM)
                pass
        return "Finished"


    def ping(self,device, timeout):
        p = Popen(['timeout', str(timeout), 'ping' ,device], stdout=PIPE)
        time.sleep(timeout + 0.1)
        connected = self.parse_output_to_conn(p.stdout.read())
        return connected

    def parse_output_to_conn(self,string):
        if string.find("time") >= 0:
            connected = True
        else:
            connected = False
        return connected

    def print_output(self, s):
        print(s)

    def thread_complete(self):
        print("THREAD COMPLETE!")

    def closeEvent(self, event):
        sys.exit(0)

    def create_node(self):
        rospy.init_node("rr_viz")


pid = 0
if __name__ == '__main__':

    app = QApplication(sys.argv)
    print("APPLICATION PID!" + str(app.applicationPid()))
    pid = app.applicationPid()
    main_window = RossRoboticsRViz()
    main_window.show()

    signal.signal(signal.SIGINT, signal.SIG_DFL)
    sys.exit(app.exec_())
