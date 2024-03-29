#!/usr/bin/env python
from PyQt5.QtWidgets import QVBoxLayout, QHBoxLayout, QWidget, QLabel, QGridLayout, QPushButton, QFrame, QMessageBox, QInputDialog
from PyQt5.QtGui import QFont
from PyQt5.QtCore import Qt
from PyQt5 import QtCore

from ross.mission_editor.pathing.QWaypointListWidget import QWaypointListWidget
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from rr_custom_msgs.msg import TaskWaypoint as TaskWaypointMsg
import managers.file_management as file_management
from helpers import rr_qt_helper
import string

from rr_custom_msgs.srv import BuildBT
from mission_actions import BuildBTAction, WaypointMoveBaseAction
from ..estop.emergency_stop import EmergencyStop

class MissionEditor(QWidget):
    # Set up signals
    spawn_waypoint_signal = QtCore.pyqtSignal(PoseWithCovarianceStamped)
    set_enable_go_to_buttons = QtCore.pyqtSignal(bool)
    set_enable_send_mission = QtCore.pyqtSignal(bool)
   
    def __init__(self, parent):
        super(QWidget, self).__init__(parent)

        self.robot_pose = None
        
        # Set up topic subsribers
        self.robot_pose_sub = rospy.Subscriber("/robot_pose", PoseStamped, self.robot_pose_cb)
        self.spawn_waypoint_sub = rospy.Subscriber("spawn_waypoint", PoseWithCovarianceStamped, lambda msg: self.spawn_waypoint_signal.emit(msg))
        
        # Connect signal
        self.spawn_waypoint_signal.connect(self.spawn_waypoint) 

        self.v_layout = QVBoxLayout(self)

        # Title set up
        self.title_label = QLabel('Mission Editor')
        self.title_label.setFont(QFont('Ubuntu', 11, QFont.Bold))
        self.title_label.setAlignment(Qt.AlignRight)
        self.v_layout.addWidget(self.title_label)

        # Title set up
        self.editor_title_label = QLabel('Editor:')
        self.editor_title_label.setFont(QFont('Ubuntu', 10, QFont.Bold))
        self.v_layout.addWidget(self.editor_title_label)

        # Waypoint list
        self.waypoint_list = QWaypointListWidget()
        self.v_layout.addWidget(self.waypoint_list, 5)

        # Buttons
        self.add_here_button = QPushButton('Add Here')
        self.duplicate_button = QPushButton('Duplicate')
        self.delete_button = QPushButton('Delete')
        self.delete_all_button = QPushButton('Delete All')
        self.save_mission_button = QPushButton('Save Mission')
        self.load_mission_button = QPushButton('Load Mission')

        self.add_here_button.pressed.connect(self.add_here)
        self.duplicate_button.pressed.connect(self.duplicate)
        self.delete_button.pressed.connect(self.delete)
        self.delete_all_button.pressed.connect(self.delete_all)
        self.save_mission_button.pressed.connect(self.save)
        self.load_mission_button.pressed.connect(self.load)

        self.button_layout = QGridLayout()
        self.button_layout.addWidget(self.add_here_button, 0, 0)
        self.button_layout.addWidget(self.duplicate_button, 0, 1)
        self.button_layout.addWidget(self.delete_button, 1, 0)
        self.button_layout.addWidget(self.delete_all_button, 1, 1)
        self.button_layout.addWidget(self.save_mission_button, 2, 0)
        self.button_layout.addWidget(self.load_mission_button, 2, 1)
        self.v_layout.addLayout(self.button_layout)

        # Line
        self.line = QFrame()
        self.line.setFrameShape(QFrame.HLine)
        self.line.setFrameShadow(QFrame.Sunken)
        self.v_layout.addWidget(self.line)

        # Send command
        self.command_title = QLabel('Send Command:')
        self.command_title.setFont(QFont('Ubuntu', 10, QFont.Bold))
        self.v_layout.addWidget(self.command_title)

        # Command buttons
        self.h_layout_command = QHBoxLayout()

        self.go_to_selected_button = QPushButton('Go to Selected')
        self.go_to_selected_button.pressed.connect(self.go_to_selected)
        
        self.go_to_all_button = QPushButton('Go to All')
        self.go_to_all_button.pressed.connect(self.go_to_all)

        self.enable_go_to_buttons(False)
        self.set_enable_go_to_buttons.connect(self.enable_go_to_buttons)

        self.send_mission_button = QPushButton('Send Mission')
        self.send_mission_button.pressed.connect(self.send_mission)
        self.enable_send_mission_button(False)
        self.set_enable_send_mission.connect(self.enable_send_mission_button)

        self.h_layout_command.addWidget(self.go_to_selected_button)
        self.h_layout_command.addWidget(self.go_to_all_button)
        self.h_layout_command.addWidget(self.send_mission_button)

        self.v_layout.addLayout(self.h_layout_command)

        self.emergency_stop_widget = EmergencyStop(self)
        self.v_layout.addWidget(self.emergency_stop_widget)

        self.setLayout(self.v_layout)

        self.bt_action = BuildBTAction()
        self.mb_action = WaypointMoveBaseAction()
        
        # Setup state checker:
        self.bt_state_checker = rr_qt_helper.StateCheckerTimer(
            self.bt_action.is_connected,  self.set_enable_send_mission, Hz=1./3.)
        self.bt_state_checker.start()
        self.mb_state_checker = rr_qt_helper.StateCheckerTimer(
            self.mb_action.is_connected,  self.set_enable_go_to_buttons, Hz=1./3.)
        self.mb_state_checker.start()

    def robot_pose_cb(self, msg):
        self.robot_pose = msg

    def spawn_waypoint(self, pwcs_msg):
        msg = TaskWaypointMsg()
        pose_stamped = PoseStamped()
        pose_stamped.header = pwcs_msg.header
        pose_stamped.pose = pwcs_msg.pose.pose
        msg.pose_stamped = pose_stamped
        self.waypoint_list.append(self.waypoint_list.new_waypoint(msg))

    def add_here(self):
        if self.robot_pose:
            msg = PoseWithCovarianceStamped()
            msg.header = self.robot_pose.header
            msg.pose.pose = self.robot_pose.pose
            self.spawn_waypoint_signal.emit(msg)
        else:
            rospy.logwarn("Robot pose not connected")

    def duplicate(self):
        if self.waypoint_list.get_selected_wp():
            self.waypoint_list.duplicate(self.waypoint_list.get_selected_wp())

    def delete(self):
        if self.waypoint_list.get_selected_wp():
            self.waypoint_list.remove(self.waypoint_list.get_selected_wp())        

    def delete_all(self):
        for _ in range(self.waypoint_list.len()):
            self.waypoint_list.remove(
                self.waypoint_list.get_wp(self.waypoint_list.len()-1))

    def save(self):
        mission_name, ok = QInputDialog.getText(self, "Mission file name", "Specify file name to save mission:")

        if ok:
            if mission_name == "":
                mission_name = "default"
            if mission_name[-5:] == ".yaml":
                mission_name = mission_name[0:-5]

            mission_name = string.replace(mission_name, '.', '_')
            mission_name = mission_name + ".yaml"
                        
            save_path = file_management.get_mission_files_dir() + "/" + mission_name            
            self.waypoint_list.saveToPath(save_path, "Mission" + str(rospy.Time.now()))

    def load(self):
        mission_files = file_management.get_files(file_management.get_mission_files_dir(), ".yaml")
        
        if mission_files == []:
            msg = QMessageBox()
            msg.setWindowTitle("Load Mission")
            msg.setIcon(QMessageBox.Information)
            msg.setText("No mission files available")
            msg.setStandardButtons(QMessageBox.Ok)
            msg.exec_()
        else:
            mission_file, ok = QInputDialog.getItem(self, "Select mission to load", "Available missions:", mission_files, 0, False)
        
            if ok:
                load_path = file_management.get_mission_files_dir() + "/" + mission_file
                if load_path[-5:] != ".yaml":
                    load_path = load_path + ".yaml"
                
                self.waypoint_list.loadFromPath(load_path)         

    def enable_go_to_buttons(self, enabled):
        self.go_to_selected_button.setEnabled(enabled)
        self.go_to_all_button.setEnabled(enabled)

    def enable_send_mission_button(self, enabled):
        self.send_mission_button.setEnabled(enabled)

    def go_to_selected(self):
        self.mb_action.goto_action(self.waypoint_list)

    def go_to_all(self):
        self.mb_action.gotoall_action(self.waypoint_list)

    def send_mission(self):
        self.bt_action.build_bt_action(self.waypoint_list)
