import os
import rospy
import rospkg
import tf
import random
import string
import time
import operator
from datetime import datetime, date
from geometry_msgs.msg import PoseWithCovarianceStamped
from functools import partial
from PyQt5 import QtGui, QtWidgets, uic, QtCore
from PyQt5.QtWidgets import QApplication, QWidget, QInputDialog, QLineEdit, QFileDialog, QMessageBox, QListWidgetItem
from std_srvs.srv import Trigger, TriggerResponse, TriggerRequest
from rr_custom_msgs.srv import String, StringResponse, StringRequest
from rr_custom_msgs.msg import StringArray
from helpers import rr_qt_helper

current_dir = os.path.dirname(os.path.abspath(__file__))
Form, Base = uic.loadUiType(os.path.join(current_dir, "slam_supervisor.ui"))


class SlamSupervisorWidget(Base, Form):
    set_enabled = QtCore.pyqtSignal(bool)

    def __init__(self, parent=None):
        super(self.__class__, self).__init__(parent)
        self.setAttribute(QtCore.Qt.WA_StyledBackground)
        self.setupUi(self)
        print("SlamSupervisorWidget Loaded")
        # Get slam_supervisor_node:
        self.slam_sup_name = rospy.get_param(
            "slam_supervisor_name", "slam_supervisor")
        self.active_nodes_sub = rospy.Subscriber(
            self.slam_sup_name+"/active_nodes", StringArray, self.active_nodes_sub_cb)
        self.active_nodes = []

        # Setup state checker:
        self.setEnabled(False)
        self.set_enabled.connect(self.setEnabled)
        self.state_checker = rr_qt_helper.StateCheckerTimer(
            self.is_slam_supervisor_up, self.set_enabled, Hz=1./3.)
        self.state_checker.start()
        # Setting up services:
        self.slam_killnodes_srv = rospy.ServiceProxy(
            self.slam_sup_name+"/kill_nodes", Trigger)
        self.slam_launch_mapping_srv = rospy.ServiceProxy(
            self.slam_sup_name+"/launch_mapping", Trigger)
        self.slam_launch_localization_srv = rospy.ServiceProxy(
            self.slam_sup_name+"/launch_localization", String)
        self.slam_list_maps_srv = rospy.ServiceProxy(
            self.slam_sup_name+"/list_maps", Trigger)
        self.slam_save_map_srv = rospy.ServiceProxy(
            self.slam_sup_name+"/save_map", String)
        self.slam_delete_map_srv = rospy.ServiceProxy(
            self.slam_sup_name+"/delete_map", String)
        self.slam_save_map_image_srv = rospy.ServiceProxy(
            self.slam_sup_name+"/save_map_image", String)

        self.default_map_name = rospy.get_param("~default_map_name", "")
        self.slam_package = rospy.get_param(self.slam_sup_name+'/slam_package',"")

        # Setup rostopiclabel
        self.modeLabel.setup(self.slam_sup_name+"/mode")
        self.initial_mode = rospy.get_param(self.slam_sup_name+"/slam_mode","")
        self.modeLabel.setText(self.initial_mode.capitalize())

        self.loadedMapLabel.setup(self.slam_sup_name+"/default_map_path")
        if(self.initial_mode == 'localization'):
            loaded_map_path = rospy.get_param(self.slam_sup_name+"/default_map_path","")
            loaded_map_path_split = loaded_map_path.split('/')
            x = len(loaded_map_path_split)
            self.loaded_map_name = loaded_map_path_split[x-1]
            self.loadedMapLabel.setText(self.loaded_map_name)

        # Connecting buttons:
        self.switchToMappingButton.pressed.connect(self.switchToMappingSlot)
        self.switchToLocalizationButton.pressed.connect(
            self.switchToLocalizationSlot)

        self.deleteMapButton.pressed.connect(self.delete_map_slot)
        self.saveMapImage.pressed.connect(self.save_map_image)
        # self.saveLocally.pressed.connect(self.saveLocallySlot)
        # self.saveLocally.setEnabled(False)
        self.saveNav.pressed.connect(self.saveNavSlot)
        # Map update:
        self.map_list_update_timer = QtCore.QTimer(self)
        self.map_list_update_timer.timeout.connect(self.map_list_update)
        self.map_list_update_timer.start(1000)

    def is_slam_supervisor_up(self):
        if rospy.is_shutdown():
            return False
        try:
            rospy.wait_for_service(
                self.slam_sup_name+"/kill_nodes", rospy.Duration(3))
            return True
        except:
            # Exit if theres no service
            return False

    def active_nodes_sub_cb(self, msg):
        self.active_nodes = msg.data

    def switchToMappingSlot(self):
        trig_resp = self.slam_killnodes_srv.call(TriggerRequest())
        if trig_resp.success:
            print(trig_resp.message)
            temp_timer = rospy.Timer(
                rospy.Duration(.1), lambda _: self.waitTillDead_and_execute(self.runMapping), oneshot=True)
        else:
            print("failed calling slam_killnodes_srv")

    def runMapping(self):
        trig_resp = self.slam_launch_mapping_srv.call(TriggerRequest())
        if trig_resp.success:
            print(trig_resp.message)
            self.modeLabel.setText("Mapping")
            self.loadedMapLabel.setText("")
            self.loaded_map_name=""
        else:
            print("failedcalling slam_launch_mapping_srv")

    def switchToLocalizationSlot(self):
        trig_resp = self.slam_killnodes_srv.call(TriggerRequest())
        if trig_resp.success:
            print(trig_resp.message)
            temp_timer = rospy.Timer(
                rospy.Duration(.1), lambda _: self.waitTillDead_and_execute(self.runLocalization), oneshot=True)
        else:
            print("failed calling slam_killnodes_srv")

    def runLocalization(self):
        _str = StringRequest()
        _str.str = self.mapListWidget.currentItem().text().split(".")[0].strip()
        trig_resp = self.slam_launch_localization_srv.call(_str)
        if trig_resp.success:
            print(trig_resp.message)
            self.modeLabel.setText("Localization")
            self.loaded_map_name = _str.str
            self.loadedMapLabel.setText(self.loaded_map_name)
        else:
            print("failed calling slam_launch_localization_srv")

    # def saveLocallySlot(self):
    #     rospy.loginfo("saveLocallySlot")
    #     pass

    def waitTillDead_and_execute(self, func):
        if len(self.active_nodes) == 0:
            # print("!!! Timer found nodes 0")
            func()
        else:
            # print("!!! Timer found nodes >0 , new timer created")
            rospy.Timer(
                rospy.Duration(1), lambda _: self.waitTillDead_and_execute(func), oneshot=True)

    def saveNavSlot(self):
        _str = StringRequest()
        map_name = self.mapName.text()
        if map_name !='':
            _str.str = map_name
        else:
            if not self.default_map_name:
                _str.str = randomTimeString()
            else:
                _str.str = self.default_map_name
        trig_resp = self.slam_save_map_srv.call(_str)
        if trig_resp.success:
            print(trig_resp.message)
            self.mapName.clear()
        else:
            print("failed calling slam_save_map_srv")
            print(trig_resp.message)

    def map_list_handle(self, remote_list):
        local_maps = []
        eligible_maps = []
        for index in range(self.mapListWidget.count()):
            local_maps.append(self.mapListWidget.item(index).text())
        # print("local: {}".format(local_maps))
        # print("remote: {}".format(remote_list))

        for _map in remote_list:
            if(self.slam_package == "slam_toolbox") and ".posegraph" in _map:
                eligible_maps.append(_map)
            elif(self.slam_package == "iris_lama") and ".yaml" in _map:
                eligible_maps.append(_map)
            elif(self.slam_package == "rtabmap") and ".db" in _map:
                eligible_maps.append(_map)


        for _map in local_maps:
            if _map not in eligible_maps:
                self.mapListWidget.takeItem(local_maps.index(_map))
                local_maps.remove(_map)
        # print("local: {}".format(local_maps))
        for _map in eligible_maps:
            if _map not in local_maps:
                self.mapListWidget.addItem(_map)

    def map_list_update(self):
        if self.isEnabled() and not rospy.is_shutdown():
            try:
                trig_resp = self.slam_list_maps_srv.call(TriggerRequest())
            except Exception as e:
                rospy.logwarn_throttle(
                    10, "failed to fetch maps: {}".format(e))
                return
            current_item = self.mapListWidget.currentItem()
            self.switchToLocalizationButton.setEnabled(
                current_item is not None)
            if trig_resp.success:
                # print(trig_resp.message)
                remote_maps = str(trig_resp.message).split(",")
                # remote_maps = [map.strip() for map in remote_maps]#Do not strip off filetypes
                try:
                    sorted_remote_maps=sorted(remote_maps, key=lambda x: (x[x.index(".")+1] ,x[1])) #sort by filetype first then alphabetically
                    self.map_list_handle(sorted_remote_maps)
                except:
                    rospy.logwarn_throttle(10, "No maps in the maps folder.")
            else:
                rospy.logwarn_throttle(10, "failed to fetch maps")

    def delete_map_slot(self):
        _str = StringRequest()
        _str.str = self.mapListWidget.currentItem().text().split(".")[0].strip()
        try:
            trig_resp = self.slam_list_maps_srv.call(TriggerRequest())
        except Exception as e:
            rospy.logwarn_throttle(
                10, "failed to fetch maps: {}".format(e))
            return
        current_item = self.mapListWidget.currentItem()
        self.switchToLocalizationButton.setEnabled(
            current_item is not None)
        if trig_resp.success:
            # print(trig_resp.message)
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
                trig_resp = self.slam_delete_map_srv.call(_str)
                if trig_resp.success:
                    print(trig_resp.message)
                else:
                    print("failed calling slam delete map service")
                    print(trig_resp.message)

    def save_map_image(self):
        _str = StringRequest()
        map_name = self.mapName.text()
        if map_name !='':
            _str.str = map_name
        else:
            if not self.default_map_name:
                _str.str = randomTimeString()
            else:
                _str.str = self.default_map_name

        try:
            self.rospack.get_path('rr_ocu')
            print("found")
        except:
            print("not found")
        # trig_resp = self.slam_save_map_image_srv.call(_str)
        # if trig_resp.success:
        #     print(trig_resp.message)
        #     self.mapName.clear()
        # else:
        #     print("failed calling slam_save_map_image_srv")
        #     print(trig_resp.message)


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


# Services:
# * ~kill_nodes - Trigger to kill all the nodes matching regex (.*slam)
# * ~launch_mapping - Trigger to launch mapping node
# * ~launch_localization - String to launch localization. Provide empty string to launch the default map, if using slam_toolbox provide the node name without suffix (e.g. `map`). If using any other SLAM package provide the map name with suffix (e.g. map.yaml)
# * ~list_maps - Trigger that returns all the map files in the map directory
# * ~save_map - String that saves map with a default name


# Services:
# * ~kill_nodes - Trigger, kill all nodes matching a regex
# * ~launch_node - Trigger, start the launch_file
# * ~purge_bags - Trigger, removes ALL bag files in the bag directory
# * ~purge_single_bag - RemoveFileSrv, removes a single bag, requires an absolute path to the file
