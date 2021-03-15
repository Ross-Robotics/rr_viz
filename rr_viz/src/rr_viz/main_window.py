#!/usr/bin/env python
import sys

from PyQt5.QtWidgets import QApplication, QMainWindow

from main_window_tabs import *

class MainWindow(QMainWindow):
    def __init__(self, parent=None):
        super(QMainWindow, self).__init__(parent)
        self.setWindowTitle("Test")
        self.x = 10
        self.y = 10
        self.width = 1482
        self.height = 868
        self.setGeometry(self.x, self.y, self.width, self.height)
        self.tabWidget = MainWindowTabs(self)
        self.setCentralWidget(self.tabWidget)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    main_window = MainWindow()
    main_window.show()
    sys.exit(app.exec_())

