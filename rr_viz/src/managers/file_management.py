#!/usr/bin/env python
import os
import rospy
import rospkg
from PyQt5.QtWidgets import QApplication, QWidget, QInputDialog, QLineEdit, QFileDialog, QMessageBox

RR_VIZ_USER_DIR = os.path.expanduser("~/Desktop/RossRobotics")
RR_VIZ_MISSION_DIR = RR_VIZ_USER_DIR + "/mission"
RR_VIZ_MAP_IMAGES_DIR = RR_VIZ_USER_DIR + "/map_images"

def get_rrviz_dir():
    rospack = rospkg.RosPack()
    return rospack.get_path('rr_viz')

def get_rrviz_resdir():
    return get_rrviz_dir() + "/res"

def get_rrviz_cfgdir():
    return get_rrviz_resdir() + "/cfg"

def get_mission_files_dir():
    check_user_dir()
    return RR_VIZ_MISSION_DIR

def get_map_images_dir():
    check_user_dir()
    return RR_VIZ_MAP_IMAGES_DIR

def check_user_dir():
    # Check if  the ~/.rr directory is setup and if it isnt, set it up.
    if not os.path.exists(RR_VIZ_USER_DIR):
        create_ross_robotics_dir()
        rospy.logerr(RR_VIZ_USER_DIR + " directory did not exist, and has been created. It is needed for RR_VIZ to work.")
    if not os.path.exists(RR_VIZ_MISSION_DIR):
        create_mission_dir()
        rospy.logerr(RR_VIZ_MISSION_DIR + " directory did not exist, and has been created. It is needed for RR_VIZ to work.")
    if not os.path.exists(RR_VIZ_MAP_IMAGES_DIR):
        create_map_images_dir()
        rospy.logerr(RR_VIZ_MAP_IMAGES_DIR + " directory did not exist, and has been created. It is needed for RR_VIZ to work.")

def get_files(dir_path, extension):
    if not os.path.exists(dir_path):
        raise OSError("The specified path, %s, does not exist" % dir_path)

    files = []
    for f in os.listdir(dir_path):
      if f.endswith(extension):
        files.append(f)
    return files

def create_ross_robotics_dir():
    os.mkdir(RR_VIZ_USER_DIR)

def create_mission_dir():
    os.mkdir(RR_VIZ_MISSION_DIR)

def create_map_images_dir():
    os.mkdir(RR_VIZ_MAP_IMAGES_DIR) 
