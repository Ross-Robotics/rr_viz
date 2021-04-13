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
    return get_user_dir() + "/mission"

def get_map_images_dir():
    return get_user_dir() + "/map_images"

def is_userdir():
    # Check if  the ~/.rr directory is setup and if it isnt, set it up.
    is_setup = False
    def msgbtn(i):
        if "Yes" in i.text():
            try:
                os.mkdir(RR_VIZ_USER_DIR)
                os.mkdir(RR_VIZ_USER_DIR +"/misison")
                os.mkdir(RR_VIZ_USER_DIR + "/map_images")                
            except OSError:
                print ("Creation of the directory %s failed" %
                       RR_VIZ_USER_DIR)
            else:
                is_setup = True
                print ("Successfully created the directory %s " %
                       RR_VIZ_USER_DIR)

    if not os.path.exists(RR_VIZ_USER_DIR):
        msg = QMessageBox()
        msg.setIcon(QMessageBox.Warning)
        msg.setText("This user does not have Ross Robotics directory set up. Would you like it set up now? (creates a folder at ~/Desktop/RossRobotics to store local files)")
        msg.setWindowTitle("RR directory setup")
        msg.setStandardButtons(QMessageBox.Yes | QMessageBox.No)
        msg.buttonClicked.connect(msgbtn)
        msg.exec_()
        return is_setup
    else:
        return True

def get_user_dir(terminate=False):
    if is_userdir():       
        return RR_VIZ_USER_DIR
    elif terminate:
        return None
    else:
        return get_user_dir(terminate=True)#Call recursively to avoid errors