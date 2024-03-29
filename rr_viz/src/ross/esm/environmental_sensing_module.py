#!/usr/bin/env python
from PyQt5.QtWidgets import QTabWidget, QVBoxLayout, QWidget, QHBoxLayout, QLabel
from PyQt5.QtGui import QFont

import rospy
from std_msgs.msg import String
from rr_custom_msgs.msg import Float32Stamped
from sensor_msgs.msg import RelativeHumidity, FluidPressure, Temperature, Illuminance

class EnvironmentalSensingModule(QWidget):
    def __init__(self, parent):
        super(QWidget, self).__init__(parent)

        self.v_layout = QVBoxLayout(self)
        # Set up ESM info 
        self.title_label = QLabel('Environmental Sensing Module data:')
        self.title_label.setFont(QFont('Ubuntu', 11, QFont.Bold))
        self.v_layout.addWidget(self.title_label)

        self.h_layout = QHBoxLayout(self)

        self.ir_data_label = QLabel('IR: ')
        self.ir_data_label.setFont(QFont('Ubuntu', 11))

        self.humidity_data_label = QLabel('Humidity: ')
        self.humidity_data_label.setFont(QFont('Ubuntu', 11))

        self.pressure_data_label = QLabel('Pressure: ')
        self.pressure_data_label.setFont(QFont('Ubuntu', 11))

        self.temp_data_label = QLabel('Temp: ')
        self.temp_data_label.setFont(QFont('Ubuntu', 11))

        self.uv_data_label = QLabel('UV: ')
        self.uv_data_label.setFont(QFont('Ubuntu', 11))

        self.vis_data_label = QLabel('Visible light: ')
        self.vis_data_label.setFont(QFont('Ubuntu', 11))

        self.h_layout.addWidget(self.ir_data_label)
        self.h_layout.addWidget(self.humidity_data_label)
        self.h_layout.addWidget(self.pressure_data_label)
        self.h_layout.addWidget(self.temp_data_label)
        self.h_layout.addWidget(self.uv_data_label)
        self.h_layout.addWidget(self.vis_data_label)

        self.v_layout.addLayout(self.h_layout)

        self.setLayout(self.v_layout)

        # Setup topic subscribers
        self.esm_namespace = rospy.get_param("esm_namespace", "mk3_esm")
        self.ir_topic_name = "/" + self.esm_namespace + "/environmental_sensor/IR"
        self.ir_topic_sub = rospy.Subscriber(self.ir_topic_name, Illuminance, self.ir_update)

        self.humidity_topic_name = "/" + self.esm_namespace + "/environmental_sensor/humidity"
        self.humidity_topic_sub = rospy.Subscriber(self.humidity_topic_name, RelativeHumidity, self.humidity_update)

        self.pressure_topic_name = "/" + self.esm_namespace + "/environmental_sensor/pressure"
        self.pressure_topic_sub = rospy.Subscriber(self.pressure_topic_name, FluidPressure, self.pressure_update)

        self.temp_topic_name = "/" + self.esm_namespace + "/environmental_sensor/temperature"
        self.temp_topic_sub = rospy.Subscriber(self.temp_topic_name, Temperature, self.temp_update)

        self.uv_topic_name = "/" + self.esm_namespace + "/environmental_sensor/uv_index"
        self.uv_topic_sub = rospy.Subscriber(self.uv_topic_name, Float32Stamped, self.uv_update)

        self.vis_topic_name = "/" + self.esm_namespace + "/environmental_sensor/visible_light"
        self.vis_topic_sub = rospy.Subscriber(self.vis_topic_name, Illuminance, self.vis_update)

        self.degree_symbol = u'\N{DEGREE SIGN}'
        
    def ir_update(self, msg):
        self.ir_data_label.setText("IR: " + format(msg.illuminance, ".2f") + " Lux")

    def humidity_update(self, msg):
        self.humidity_data_label.setText("Humidity: " + format(msg.relative_humidity, ".2f") + "%")

    def pressure_update(self, msg):
        self.pressure_data_label.setText("Pressure: " + format(msg.fluid_pressure, ".2f") + " Pascals")

    def temp_update(self, msg):
        self.temp_data_label.setText("Temp: " + format(msg.temperature, ".2f") + self.degree_symbol + "C")

    def uv_update(self, msg):
        self.uv_data_label.setText("UV index: " + format(msg.data, ".2f"))

    def vis_update(self, msg):
        self.vis_data_label.setText("Visible light: " + format(msg.illuminance, ".2f") + " Lux")
