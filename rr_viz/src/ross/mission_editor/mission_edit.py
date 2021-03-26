# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'mission_editor.ui'
#
# Created by: PyQt5 UI code generator 5.10.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_Form(object):
    def setupUi(self, Form):
        Form.setObjectName("Form")
        Form.resize(477, 519)
        self.verticalLayout = QtWidgets.QVBoxLayout(Form)
        self.verticalLayout.setObjectName("verticalLayout")
        self.verticalLayout_3 = QtWidgets.QVBoxLayout()
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.label = QtWidgets.QLabel(Form)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label.setFont(font)
        self.label.setObjectName("label")
        self.verticalLayout_3.addWidget(self.label)
        self.waypointList = QWaypointListWidget(Form)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.waypointList.sizePolicy().hasHeightForWidth())
        self.waypointList.setSizePolicy(sizePolicy)
        self.waypointList.setMinimumSize(QtCore.QSize(400, 0))
        self.waypointList.setObjectName("waypointList")
        self.verticalLayout_3.addWidget(self.waypointList)
        self.button_holder = QtWidgets.QVBoxLayout()
        self.button_holder.setObjectName("button_holder")
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.button_holder.addLayout(self.horizontalLayout)
        self.horizontalLayout_4 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_4.setObjectName("horizontalLayout_4")
        self.addButton = QtWidgets.QPushButton(Form)
        self.addButton.setObjectName("addButton")
        self.horizontalLayout_4.addWidget(self.addButton)
        self.duplicateButton = QtWidgets.QPushButton(Form)
        self.duplicateButton.setObjectName("duplicateButton")
        self.horizontalLayout_4.addWidget(self.duplicateButton)
        self.button_holder.addLayout(self.horizontalLayout_4)
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.deleteAllButton = QtWidgets.QPushButton(Form)
        self.deleteAllButton.setObjectName("deleteAllButton")
        self.horizontalLayout_3.addWidget(self.deleteAllButton)
        self.deleteButton = QtWidgets.QPushButton(Form)
        self.deleteButton.setObjectName("deleteButton")
        self.horizontalLayout_3.addWidget(self.deleteButton)
        self.button_holder.addLayout(self.horizontalLayout_3)
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.saveButton = QtWidgets.QPushButton(Form)
        self.saveButton.setObjectName("saveButton")
        self.horizontalLayout_2.addWidget(self.saveButton)
        self.loadButton = QtWidgets.QPushButton(Form)
        self.loadButton.setObjectName("loadButton")
        self.horizontalLayout_2.addWidget(self.loadButton)
        self.button_holder.addLayout(self.horizontalLayout_2)
        self.verticalLayout_3.addLayout(self.button_holder)
        self.verticalLayout.addLayout(self.verticalLayout_3)

        self.retranslateUi(Form)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        _translate = QtCore.QCoreApplication.translate
        Form.setWindowTitle(_translate("Form", "Form"))
        self.label.setText(_translate("Form", "Mission Editor:"))
        self.addButton.setText(_translate("Form", "Add Here"))
        self.duplicateButton.setText(_translate("Form", "Duplicate"))
        self.deleteAllButton.setText(_translate("Form", "Delete All"))
        self.deleteButton.setText(_translate("Form", "Delete"))
        self.saveButton.setText(_translate("Form", "Save Mission"))
        self.loadButton.setText(_translate("Form", "Load Mission"))

from pathing.QWaypointListWidget import QWaypointListWidget

if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    Form = QtWidgets.QWidget()
    ui = Ui_Form()
    ui.setupUi(Form)
    Form.show()
    sys.exit(app.exec_())

