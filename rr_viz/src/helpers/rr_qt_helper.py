import rospy
from PyQt5.QtCore import QThread
"""[This module is a copy from rros_gui !!! It was  made to use pyqt instead of PySide2]
    #TODO  Unite this somehow?
    Returns:
        [type]: [description]
"""


class HeteroThreadTimer(QThread):

    def __init__(self, cb=None, signal=None, Hz=1, parent=None):
        """[This is a Timer class that can handle blocking calls.  e.g. internet connectivity etc.
            Made inspired from here: https://stackoverflow.com/questions/9862172/timer-in-a-child-thread-in-pyside]

        Args:
            cb ([function], optional): [Callback to be triggered]. Defaults to None.
            signal ([pyside2 signal], optional): [pyside signal to be emmited]. Defaults to None.
            Hz (int, optional): [Frequency]. Defaults to 1."""
        super(HeteroThreadTimer, self).__init__(parent)
        self.cb = cb
        self.signal = signal
        self.Hz = Hz
        self.period = round(1000./Hz)
        self._running = False

    def run(self):
        """[The run loop. !!!Attention!!! Please use start() to run the thread. Do not use run()]
        """
        self._running = True
        while self._running:
            if self.signal:
                self.signal.emit()
            if self.cb:
                self.cb()
            self.msleep(int(self.period))

    def stop(self, wait=False):
        """[Stop the timer]

        Args:
            wait (bool, optional): [If wait=true, stop will return after run method finished executing. i.e. turns this into blocking ]. Defaults to False.
        """
        self._running = False
        if wait:
            self.wait()


class StateCheckerTimer(HeteroThreadTimer):

    def __init__(self, cb, signal, Hz=1, parent=None):
        """[State checker is a timer on a separate thread that emits a qt signal  with the output of a provided call back as input onto the signal]

        Args:
            cb (function): [description]
            signal ([type]): [description]
            Hz (int, optional): [description]. Defaults to 1.
            parent ([type], optional): [description]. Defaults to None.
        """
        if not (cb and signal):
            rospy.logerr("Cb  and signal must be provided")
            return
        super(StateCheckerTimer, self).__init__(cb, signal, Hz, parent)

    def run(self):
        """[The run loop. !!!Attention!!! Please use start() to run the thread. Do not use run()]
        """
        self._running = True
        while self._running:
            self.signal.emit(self.cb())
            self.msleep(int(self.period))
