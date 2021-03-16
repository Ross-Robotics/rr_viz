#!/usr/bin/env python
import sys
from PyQt5.QtWidgets import QVBoxLayout, QHBoxLayout, QWidget, QLabel, QPushButton, QListWidget, QLineEdit
from PyQt5.QtGui import QFont
from PyQt5.QtCore import Qt

class SlamSupervisor(QWidget):
    def __init__(self, parent):
            super(QWidget, self).__init__(parent)
            self.v_layout = QVBoxLayout()

            #Title
            self.title_label = QLabel('SLAM')
            self.title_label.setFont(QFont('Ubuntu', 11, QFont.Bold))
            self.title_label.setAlignment(Qt.AlignRight)
            self.v_layout.addWidget(self.title_label)

            #Slam mode
            self.h_layout_mode = QHBoxLayout()
            self.slam_mode_text_label = QLabel('Mode:')
            self.slam_mode_text_label.setFont(QFont('Ubuntu', 10, QFont.Bold))
            self.h_layout_mode.addWidget(self.slam_mode_text_label, 3)

            self.slam_mode_label = QLabel('')
            self.slam_mode_label.setFont(QFont('Ubuntu', 10))
            self.h_layout_mode.addWidget(self.slam_mode_label, 7)

            self.v_layout.addLayout(self.h_layout_mode)

            #Loaded map
            self.h_layout_map = QHBoxLayout()
            self.loaded_map_text_label = QLabel('Loaded map:')
            self.loaded_map_text_label.setFont(QFont('Ubuntu', 10, QFont.Bold))
            self.h_layout_map.addWidget(self.loaded_map_text_label, 3)
            
            self.loaded_map_label = QLabel('')
            self.loaded_map_label.setFont(QFont('Ubuntu', 10))
            self.h_layout_map.addWidget(self.loaded_map_label, 7)

            self.v_layout.addLayout(self.h_layout_map)

            #Selecting a map
            self.select_a_map_label = QLabel('Select a map:')
            self.select_a_map_label.setFont(QFont('Ubuntu', 11))
            self.v_layout.addWidget(self.select_a_map_label)

            self.map_list_widget = QListWidget()
            self.v_layout.addWidget(self.map_list_widget)

            #Delete map
            self.h_layout_delete_map = QHBoxLayout()
            self.delete_spacer = QLabel('')
            self.h_layout_delete_map.addWidget(self.delete_spacer, 7)
            self.delete_map_button = QPushButton('Delete Map')
            self.h_layout_delete_map.addWidget(self.delete_map_button, 3)

            self.v_layout.addLayout(self.h_layout_delete_map)

            #Localization title
            self.h_layout_localization = QHBoxLayout()
            self.localization_spacer = QLabel('')
            self.h_layout_localization.addWidget(self.localization_spacer, 7)

            self.localization_label = QLabel('Localization')
            self.localization_label.setFont(QFont('Ubuntu', 10, QFont.Bold))
            self.localization_label.setAlignment(Qt.AlignRight)
            self.h_layout_localization.addWidget(self.localization_label, 3)

            self.v_layout.addLayout(self.h_layout_localization)

            #Localization button
            self.localization_button = QPushButton('Switch to Localization')
            self.v_layout.addWidget(self.localization_button)

            #Mapping title
            self.h_layout_mapping = QHBoxLayout()
            self.mapping_spacer = QLabel('')
            self.h_layout_mapping.addWidget(self.mapping_spacer, 7)

            self.mapping_label = QLabel('Mapping')
            self.mapping_label.setFont(QFont('Ubuntu', 10, QFont.Bold))
            self.mapping_label.setAlignment(Qt.AlignRight)
            self.h_layout_mapping.addWidget(self.mapping_label)

            self.v_layout.addLayout(self.h_layout_mapping)
            
            #Mapping button
            self.mapping_button = QPushButton('Switch to Mapping')
            self.v_layout.addWidget(self.mapping_button)

            #File name
            self.h_layout_file_name = QHBoxLayout()
            self.filename_label = QLabel('Filename:')
            self.filename_label.setFont(QFont('Ubuntu', 10, QFont.Bold))
            self.h_layout_file_name.addWidget(self.filename_label, 2)

            self.file_name_text_edit = QLineEdit()
            self.file_name_text_edit.setFont(QFont('Ubuntu', 10))
            self.h_layout_file_name.addWidget(self.file_name_text_edit, 8)

            self.v_layout.addLayout(self.h_layout_file_name)

            #Save map
            self.h_layout_save_map = QHBoxLayout()
            self.save_map_button = QPushButton('Save Map')
            self.save_map_image_button = QPushButton('Save Map Image')
            self.h_layout_save_map.addWidget(self.save_map_button)
            self.h_layout_save_map.addWidget(self.save_map_image_button)

            self.v_layout.addLayout(self.h_layout_save_map)

            self.setLayout(self.v_layout)