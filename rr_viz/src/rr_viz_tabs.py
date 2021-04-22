#!/usr/bin/env python
from PyQt5.QtWidgets import QTabWidget, QVBoxLayout, QWidget, QHBoxLayout, QLabel

from rviz_tabs import RvizTabs
from ross.ross_robotics import RossRobotics
import managers.file_management as file_management

class RRVizTabs(QWidget):
    def __init__(self, parent):
        super(QWidget, self).__init__(parent)
        self.main_window_layout = QVBoxLayout(self)

        self.main_window_tabs = QTabWidget()
        self.ross_robotics_tab = QWidget()
        
        self.main_window_tabs.addTab(self.ross_robotics_tab, "Ross Robotics")
        
        self.ross_robotics_tab.h_layout = QHBoxLayout(self)
        self.ross_robotics_tab.v_layout1 = QVBoxLayout(self)
        self.ross_robotics_tab.v_layout2 = QVBoxLayout(self)

        self.ross_robotics_tab.v_layout1.addWidget(RvizTabs(self))
     
        self.ross_robotics_tab.v_layout2.addWidget(RossRobotics(self),7)

        self.logo_label = QLabel(self)
        logo_path = file_management.get_rrviz_resdir() + "/logo.png"
        logo_pixmap = QPixmap(logo_path)
        logo_scaled = logo_pixmap.scaledToWidth(300)
        self.logo_label.setPixmap(logo_scaled)
        
        self.logo_label.setAlignment(Qt.AlignCenter | Qt.AlignVCenter)

        self.ross_robotics_tab.v_layout2.addWidget(self.logo_label, 3)

        self.ross_robotics_tab.h_layout.addLayout(self.ross_robotics_tab.v_layout1, 7)
        self.ross_robotics_tab.h_layout.addLayout(self.ross_robotics_tab.v_layout2, 3)

        self.ross_robotics_tab.setLayout(self.ross_robotics_tab.h_layout)

        self.main_window_layout.addWidget(self.main_window_tabs)
        self.setLayout(self.main_window_layout)
