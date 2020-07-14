import os
from abc import ABCMeta
from PyQt5 import QtCore, QtGui, QtWidgets, uic
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

import rviz
import managers.file_management as file_management


class RViz_frame():
    # Notes:
    # This is a raw, frameless tviz frame to contain any rviz conifg. however as reading through these:
    # https://forum.qt.io/topic/41181/how-to-pass-a-parameter-to-a-widget-created-by-designer
    # https://forum.qt.io/topic/800/solved-how-to-see-custom-slot-in-signal-slot-editor/4
    # https://doc.qt.io/qt-5/signalsandslots.html
    # https://stackoverflow.com/questions/7964869/qt-designer-how-to-add-custom-slot-and-code-to-a-button
    # https://doc.qt.io/qt-5/designer-using-custom-widgets.html
    # I did not manager to get this to take a parameter. So, this widget will usually be used in QT via a wrapper widget that sets the paramter.
    def __init__(self):
        self.rviz_config = None
        pass

    def load_rviz_frame(self, hide_menu=True, hide_status=True, splash=""):
        if self.rviz_config is not None:
            full_cfg_name = file_management.get_rrviz_cfgdir()+"/"+self.rviz_config
            rviz_frame, rviz_manager = get_rviz(
                full_cfg_name, hide_menu, hide_status, splash)
            self.rviz_frame = rviz_frame
            self.rviz_manager = rviz_manager
            self.findChildren(QLayout, "frame")[0].addWidget(rviz_frame)
        else:
            print("attempt to load rviz config before setting it")

    def set_rviz_config(self, rviz_config):
        if rviz_config != self.rviz_config:
            self.rviz_config = rviz_config


def get_rviz(config_name, hide_menu=True, hide_status=True, splash=""):
    # rviz.VisualizationFrame is the main container widget of the
    # regular RViz application, with menus, a toolbar, a status
    # bar, and many docked subpanels.  In this example, we
    # disable everything so that the only thing visible is the 3D
    # render window.
    rviz_frame = rviz.VisualizationFrame()
    # The "splash path" is the full path of an image file which
    # gets shown during loading.  Setting it to the empty string
    # suppresses that behavior.
    rviz_frame.setSplashPath(splash)
    # VisualizationFrame.initialize() must be called before
    # VisualizationFrame.load().  In fact it must be called
    # before most interactions with RViz classes because it
    # instantiates and initializes the VisualizationManager,
    # which is the central class of RViz.
    rviz_frame.initialize()
    # The reader reads config file data into the config object.
    # VisualizationFrame reads its data from the config object.
    reader = rviz.YamlConfigReader()
    config = rviz.Config()
    reader.readFile(config, config_name)
    print("Loading rviz config: {}".format(config_name))
    rviz_frame.load(config)
    print("rviz config loaded")
    # Here we disable the menu bar (from the top), status bar
    # (from the bottom), and the "hide-docks" buttons, which are
    # the tall skinny buttons on the left and right sides of the
    # main render window.
    if hide_menu:
        rviz_frame.setMenuBar(None)
    if hide_status:
        rviz_frame.setStatusBar(None)
    # rviz_frame.setHideButtonVisibility( True )#Not even sure what this is
    # frame.getManager() returns the VisualizationManager
    # instance, which is a very central class.  It has pointers
    # to other manager objects and is generally required to make
    # any changes in an rviz instance.
    rviz_manager = rviz_frame.getManager()
    return rviz_frame, rviz_manager
