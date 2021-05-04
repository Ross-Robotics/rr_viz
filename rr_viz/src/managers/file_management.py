#!/usr/bin/env python
import os
import rospy
import rospkg
from PyQt5.QtWidgets import QApplication, QWidget, QInputDialog, QLineEdit, QFileDialog, QMessageBox

RR_VIZ_USER_DIR = os.path.expanduser("~/Desktop/RossRobotics")

def get_rrviz_dir():
    rospack = rospkg.RosPack()
    return rospack.get_path('rr_viz')

def get_rrviz_resdir():
    return get_rrviz_dir() + "/res"

def get_rrviz_cfgdir():
    return get_rrviz_resdir() + "/cfg"

def get_mission_files_dir():
    is_userdir()
    return RR_VIZ_USER_DIR + "/mission"

def get_map_images_dir():
    is_userdir()
    return RR_VIZ_USER_DIR + "/map_images"

def is_userdir():
    # Check if  the ~/.rr directory is setup and if it isnt, set it up.
    if not os.path.exists(RR_VIZ_USER_DIR):
        create_user_dir()
        rospy.logerr(RR_VIZ_USER_DIR + " directory did not exist, and has been created. It is needed for RR_VIZ to work.")

def get_files(dir_path, extension):
    if not os.path.exists(dir_path):
        raise OSError("The specified path, %s, does not exist" % dir_path)

    files = []
    for f in os.listdir(dir_path):
      if f.endswith(extension):
        files.append(f)
    return files

def create_user_dir():
    os.mkdir(RR_VIZ_USER_DIR)
    os.mkdir(RR_VIZ_USER_DIR +"/mission")
    os.mkdir(RR_VIZ_USER_DIR + "/map_images") 
