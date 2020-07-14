import os
from PyQt5 import QtGui, uic

current_dir = os.path.dirname(os.path.abspath(__file__))
Form, Base = uic.loadUiType(os.path.join(current_dir, "blank.ui"))


class BlankWidget(Base, Form):
    def __init__(self, parent=None):
        super(self.__class__, self).__init__(parent)
        self.setupUi(self)
        print("Blank created")
        # Main widget called frame
