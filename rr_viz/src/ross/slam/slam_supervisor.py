#!/usr/bin/env python
from PyQt5.QtWidgets import QVBoxLayout, QHBoxLayout, QWidget, QLabel, QPushButton, QListWidget, QMessageBox, QInputDialog
from PyQt5.QtGui import QFont
from PyQt5.QtCore import Qt
from PyQt5 import QtCore

import rospy
import string
import roslaunch
from std_srvs.srv import Trigger, TriggerRequest
from rr_custom_msgs.srv import String, StringRequest
from rr_custom_msgs.msg import StringArray
from helpers import rr_qt_helper
import managers.file_management as file_management

class SlamSupervisor(QWidget):
    set_enable_panel = QtCore.pyqtSignal(bool)

    def __init__(self, parent):
        super(QWidget, self).__init__(parent)

        # Get parameters from server
        self.slam_sup_name = rospy.get_param("slam_supervisor_name", "slam_supervisor")
        self.default_map_name = rospy.get_param("~default_map_name", "map")
        self.slam_package = rospy.get_param(self.slam_sup_name + '/slam_package', "")
        self.initial_mode = rospy.get_param(self.slam_sup_name + "/slam_mode", "")
        self.loaded_map_path = rospy.get_param(self.slam_sup_name + "/default_map_path", "")

        # Setup services
        self.kill_nodes_srv = rospy.ServiceProxy(self.slam_sup_name+"/kill_nodes", Trigger)
        self.launch_mapping_srv = rospy.ServiceProxy(self.slam_sup_name+"/launch_mapping", Trigger)
        self.launch_localization_srv = rospy.ServiceProxy(self.slam_sup_name+"/launch_localization", String)
        self.list_maps_srv = rospy.ServiceProxy(self.slam_sup_name+"/list_maps", Trigger)
        self.save_map_srv = rospy.ServiceProxy(self.slam_sup_name+"/save_map", String)
        self.delete_map_srv = rospy.ServiceProxy(self.slam_sup_name+"/delete_map", String)
        self.save_map_image_srv = rospy.ServiceProxy(self.slam_sup_name+"/save_map_image", String)
        self.reset_dose_map_srv = rospy.ServiceProxy("/costmap_value_mapper/rest_map", Trigger)
        self.active_nodes_sub = rospy.Subscriber(self.slam_sup_name+"/active_nodes", StringArray, self.active_nodes_sub_cb)
        
        self.active_nodes = []

        self.v_layout = QVBoxLayout()

        # Title
        self.title_label = QLabel('SLAM')
        self.title_label.setFont(QFont('Ubuntu', 11, QFont.Bold))
        self.title_label.setAlignment(Qt.AlignRight)
        self.v_layout.addWidget(self.title_label)

        # Slam mode
        self.h_layout_mode = QHBoxLayout()
        self.mode_text_label = QLabel('Mode:')
        self.mode_text_label.setFont(QFont('Ubuntu', 10, QFont.Bold))
        self.h_layout_mode.addWidget(self.mode_text_label, 1)

        self.mode_label = QLabel('')
        self.mode_label.setFont(QFont('Ubuntu', 10))
        self.mode_label.setText(self.initial_mode.capitalize())
        self.h_layout_mode.addWidget(self.mode_label, 9)

        self.v_layout.addLayout(self.h_layout_mode)

        # Loaded map
        self.h_layout_map = QHBoxLayout()
        self.loaded_map_text_label = QLabel('Loaded map:')
        self.loaded_map_text_label.setFont(QFont('Ubuntu', 10, QFont.Bold))
        self.h_layout_map.addWidget(self.loaded_map_text_label, 2)

        self.loaded_map_label = QLabel('')
        self.loaded_map_label.setFont(QFont('Ubuntu', 10))
        self.loaded_map_name = ""
        if(self.initial_mode == "localization"):
            loaded_map_path_split = self.loaded_map_path.split('/')
            x = len(loaded_map_path_split)
            self.loaded_map_name = loaded_map_path_split[x-1]
            self.loaded_map_label.setText(self.loaded_map_name)
        else:
            self.loaded_map_label.setText(self.loaded_map_name)

        self.h_layout_map.addWidget(self.loaded_map_label, 8)

        self.v_layout.addLayout(self.h_layout_map)

        # Selecting a map
        self.select_a_map_label = QLabel('Select a map:')
        self.select_a_map_label.setFont(QFont('Ubuntu', 11))
        self.v_layout.addWidget(self.select_a_map_label)

        self.map_list_widget = QListWidget()
        self.v_layout.addWidget(self.map_list_widget)

        # Delete map
        self.h_layout_delete_map = QHBoxLayout()

        self.delete_spacer = QLabel('')
        self.h_layout_delete_map.addWidget(self.delete_spacer, 7)
        
        self.delete_map_button = QPushButton('Delete Map')
        self.delete_map_button.pressed.connect(self.delete_map)

        self.h_layout_delete_map.addWidget(self.delete_map_button, 3)

        self.v_layout.addLayout(self.h_layout_delete_map)

        # Localization title
        self.h_layout_localization = QHBoxLayout()
        self.localization_spacer = QLabel('')
        self.h_layout_localization.addWidget(self.localization_spacer, 7)

        self.localization_label = QLabel('Localization')
        self.localization_label.setFont(QFont('Ubuntu', 10, QFont.Bold))
        self.localization_label.setAlignment(Qt.AlignRight)
        self.h_layout_localization.addWidget(self.localization_label, 3)

        self.v_layout.addLayout(self.h_layout_localization)

        # Localization button
        self.localization_button = QPushButton('Switch to Localization')
        self.localization_button.pressed.connect(self.switch_to_localization)

        self.v_layout.addWidget(self.localization_button)

        # Mapping title
        self.h_layout_mapping = QHBoxLayout()
        self.mapping_spacer = QLabel('')
        self.h_layout_mapping.addWidget(self.mapping_spacer, 7)

        self.mapping_label = QLabel('Mapping')
        self.mapping_label.setFont(QFont('Ubuntu', 10, QFont.Bold))
        self.mapping_label.setAlignment(Qt.AlignRight)
        self.h_layout_mapping.addWidget(self.mapping_label)

        self.v_layout.addLayout(self.h_layout_mapping)
        
        # Mapping button
        self.mapping_button = QPushButton('Switch to Mapping')
        self.mapping_button.pressed.connect(self.switch_to_mapping)
        self.v_layout.addWidget(self.mapping_button)

        # Save map
        self.h_layout_save_map = QHBoxLayout()

        self.save_map_button = QPushButton('Save Map')
        self.save_map_button.pressed.connect(self.save_map)

        self.save_map_image_button = QPushButton('Save Map Image')
        self.save_map_image_button.pressed.connect(self.save_map_image)

        self.h_layout_save_map.addWidget(self.save_map_button)
        self.h_layout_save_map.addWidget(self.save_map_image_button)

        self.v_layout.addLayout(self.h_layout_save_map)

        self.setLayout(self.v_layout)

        self.setEnabled(False)
        self.set_enable_panel.connect(self.setEnabled)

        # Setup state checker:
        self.state_checker = rr_qt_helper.StateCheckerTimer(
            self.is_slam_supervisor_up, self.set_enable_panel, Hz=1./3.)
        self.state_checker.start()

        # Map update
        self.map_list_update_timer = QtCore.QTimer(self)
        self.map_list_update_timer.timeout.connect(self.map_list_update)
        self.map_list_update_timer.start(1000)

    def is_slam_supervisor_up(self):
        if rospy.is_shutdown():
            return False
        try:
            rospy.wait_for_service(
                self.slam_sup_name + "/kill_nodes", rospy.Duration(3))
            return True
        except:
            # Exit if theres no service
            return False
            
    def active_nodes_sub_cb(self, msg):
        self.active_nodes = msg.data

    def switch_to_mapping(self):
        trig_resp = self.kill_nodes_srv.call(TriggerRequest())
        
        if trig_resp.success:
            rospy.loginfo(trig_resp.message)
            temp_timer = rospy.Timer(
                rospy.Duration(.1), lambda _: self.waitTillDead_and_execute(self.run_mapping), oneshot=True)
        else:
            rospy.logwarn("Failed calling slam_killnodes_srv")

        trig_resp = self.reset_dose_map_srv.call(TriggerRequest())
        if trig_resp.success:
            rospy.loginfo(trig_resp.message)
        else:
            rospy.logwarn("Failed calling reset dose map ")
    
    def run_mapping(self):
        trig_resp = self.launch_mapping_srv.call(TriggerRequest())

        if trig_resp.success:
            rospy.loginfo(trig_resp.message)
            self.mode_label.setText("Mapping")
            self.loaded_map_label.setText("")
            self.loaded_map_name = ""
        else:
            rospy.logerr("Failed calling slam_launch_mapping_srv")

    def switch_to_localization(self):
        trig_resp = self.kill_nodes_srv.call(TriggerRequest())

        if trig_resp.success:
            rospy.loginfo(trig_resp.message)
            temp_timer = rospy.Timer(
                rospy.Duration(.1), lambda _: self.waitTillDead_and_execute(self.run_localization), oneshot=True)
        else:
            rospy.logerr("Failed calling slam_killnodes_srv")

        trig_resp = self.reset_dose_map_srv.call(TriggerRequest())
        if trig_resp.success:
            rospy.loginfo(trig_resp.message)
        else:
            rospy.logwarn("Failed calling reset dose map ")

    def run_localization(self):
        _str = StringRequest()
        _str.str = self.map_list_widget.currentItem().text().split(".")[0].strip()
        trig_resp = self.launch_localization_srv.call(_str)

        if trig_resp.success:
            rospy.loginfo(trig_resp.message)
            self.mode_label.setText("Localization")
            self.loaded_map_name = _str.str
            self.loaded_map_label.setText(self.loaded_map_name)
        else:
            rospy.logerr("Failed calling slam_launch_localization_srv")
        
    def waitTillDead_and_execute(self, func):
        if len(self.active_nodes) == 0:
            func()
        else:
            rospy.Timer(
                rospy.Duration(1), lambda _: self.waitTillDead_and_execute(func), oneshot=True)

    def delete_map(self):
        _str = StringRequest()
        _str.str = self.map_list_widget.currentItem().text().split(".")[0].strip()

        try:
            trig_resp = self.list_maps_srv.call(TriggerRequest())
        except Exception as e:
            rospy.logwarn_throttle(
                10, "failed to fetch maps: {}".format(e))
            return

        current_item = self.map_list_widget.currentItem()
        self.localization_button.setEnabled(current_item is not None)

        if trig_resp.success:
            remote_maps = str(trig_resp.message).split(",")
            if (_str.str == 'default_map'):
                rospy.logwarn_throttle(10, "The default map cannot be deleted")
                self.msg_to_show= "The default_map cannot be deleted."
                self.message_popup()
            elif (_str.str == self.loaded_map_name):
                rospy.logwarn_throttle(10, "Map cannot be deleted")
                self.msg_to_show = "'" + self.loaded_map_name + "' cannot be deleted while loaded."
                self.message_popup()
            else:
                trig_resp = self.delete_map_srv.call(_str)
                rospy.loginfo(trig_resp.message)
                if not trig_resp.success:
                    rospy.logerr("Failed calling slam delete map service")
 
    def save_map(self):
        _str = StringRequest()
        map_name, ok = QInputDialog.getText(self, "Map file name", "Specify file name to save the map:")

        if ok:
            if map_name != '':
                _str.str = string.replace(map_name, '.', '_')
            else:
                _str.str = self.default_map_name

            trig_resp = self.save_map_srv.call(_str)

            rospy.loginfo(trig_resp.message)
            if not trig_resp.success:
                rospy.logerr("Failed calling slam_save_map_srv")

    def save_map_image(self):
        map_name, ok = QInputDialog.getText(self, "Map file name"," Specify file name to save the map image:")

        if ok:
            if map_name != '':
                map_name = string.replace(map_name, '.', '_')
            else:
                map_name = self.default_map_name

            save_map_image_dir = file_management.get_map_images_dir()
            map_path = save_map_image_dir + '/' +  map_name

            package = "map_server"
            executable = "map_saver"
            args = "-f " + map_path
            node = roslaunch.core.Node(package, executable, args=args, output="screen")

            launch = roslaunch.scriptapi.ROSLaunch()
            launch.start()

            process = launch.launch(node)
            r = rospy.Rate(5)

            while process.is_alive() and not rospy.is_shutdown():
                rospy.loginfo_throttle(1, "Map saver running")
                r.sleep()
            rospy.loginfo("Map saved")

    def map_list_update(self):
        if self.isEnabled() and not rospy.is_shutdown():
            try:
                trig_resp = self.list_maps_srv.call(TriggerRequest())
            except Exception as e:
                rospy.logwarn_throttle(
                    10, "Failed to fetch maps: {}".format(e))
                return
            current_item = self.map_list_widget.currentItem()

            self.localization_button.setEnabled(
                current_item is not None)

            if trig_resp.success:
                remote_maps = str(trig_resp.message).split(",")
                try:
                    sorted_remote_maps=sorted(remote_maps, key=lambda x: (x[x.index(".")+1] ,x[1])) #sort by filetype first then alphabetically
                    self.map_list_handle(sorted_remote_maps)
                except:
                    rospy.logwarn_throttle(10, "No maps in the maps folder.")
            else:
                rospy.logwarn_throttle(10, "Failed to fetch maps")

    def map_list_handle(self, remote_list):
        local_maps = []
        eligible_maps = []

        for index in range(self.map_list_widget.count()):
            local_maps.append(self.map_list_widget.item(index).text())
        
        for _map in remote_list:
            if(self.slam_package == "slam_toolbox") and ".posegraph" in _map:
                eligible_maps.append(_map)
            elif(self.slam_package == "iris_lama") and ".yaml" in _map:
                eligible_maps.append(_map)
            elif(self.slam_package == "rtabmap") and ".db" in _map:
                eligible_maps.append(_map)

        for _map in local_maps:
            if _map not in eligible_maps:
                self.map_list_widget.takeItem(local_maps.index(_map))
                local_maps.remove(_map)
        
        for _map in eligible_maps:
            if _map not in local_maps:
                self.map_list_widget.addItem(_map)

    def message_popup(self):
        msg = QMessageBox()
        msg.setText(self.msg_to_show)
        msg.exec_()

def randomString(stringLength):
    letters = string.ascii_letters
    return ''.join(random.choice(letters) for i in range(stringLength))

def randomTimeString():
    # time.ctime() # 'Mon Oct 18 13:35:29 2010'
    # ' 1:36PM EDT on Oct 18, 2010'
    return "{:%m_%d_%H_%M_}".format(datetime.now())+randomString(4)
