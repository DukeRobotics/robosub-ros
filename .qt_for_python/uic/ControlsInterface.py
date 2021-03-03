# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'ControlsInterface.ui'
##
## Created by: Qt User Interface Compiler version 5.15.2
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide2.QtCore import *
from PySide2.QtGui import *
from PySide2.QtWidgets import *


class Ui_testbuttons(object):
    def setupUi(self, testbuttons):
        if not testbuttons.objectName():
            testbuttons.setObjectName(u"testbuttons")
        testbuttons.resize(321, 361)
        testbuttons.setMinimumSize(QSize(321, 361))
        self.pose_box = QGroupBox(testbuttons)
        self.pose_box.setObjectName(u"pose_box")
        self.pose_box.setGeometry(QRect(20, 80, 281, 121))
        self.layoutWidget = QWidget(self.pose_box)
        self.layoutWidget.setObjectName(u"layoutWidget")
        self.layoutWidget.setGeometry(QRect(10, 30, 274, 81))
        self.pose_grid_layout = QGridLayout(self.layoutWidget)
        self.pose_grid_layout.setSpacing(25)
        self.pose_grid_layout.setObjectName(u"pose_grid_layout")
        self.pose_grid_layout.setContentsMargins(0, 0, 0, 0)
        self.gridLayout = QGridLayout()
        self.gridLayout.setObjectName(u"gridLayout")
        self.x_pose_label = QLabel(self.layoutWidget)
        self.x_pose_label.setObjectName(u"x_pose_label")

        self.gridLayout.addWidget(self.x_pose_label, 0, 0, 1, 1)

        self.x_pose = QDoubleSpinBox(self.layoutWidget)
        self.x_pose.setObjectName(u"x_pose")

        self.gridLayout.addWidget(self.x_pose, 0, 1, 1, 1)


        self.pose_grid_layout.addLayout(self.gridLayout, 0, 0, 1, 1)

        self.gridLayout_2 = QGridLayout()
        self.gridLayout_2.setObjectName(u"gridLayout_2")
        self.y_pose_label = QLabel(self.layoutWidget)
        self.y_pose_label.setObjectName(u"y_pose_label")

        self.gridLayout_2.addWidget(self.y_pose_label, 0, 0, 1, 1)

        self.y_pose = QDoubleSpinBox(self.layoutWidget)
        self.y_pose.setObjectName(u"y_pose")

        self.gridLayout_2.addWidget(self.y_pose, 0, 1, 1, 1)


        self.pose_grid_layout.addLayout(self.gridLayout_2, 0, 1, 1, 1)

        self.gridLayout_3 = QGridLayout()
        self.gridLayout_3.setObjectName(u"gridLayout_3")
        self.z_pose_label = QLabel(self.layoutWidget)
        self.z_pose_label.setObjectName(u"z_pose_label")

        self.gridLayout_3.addWidget(self.z_pose_label, 0, 0, 1, 1)

        self.z_pose = QDoubleSpinBox(self.layoutWidget)
        self.z_pose.setObjectName(u"z_pose")

        self.gridLayout_3.addWidget(self.z_pose, 0, 1, 1, 1)


        self.pose_grid_layout.addLayout(self.gridLayout_3, 0, 2, 1, 1)

        self.gridLayout_4 = QGridLayout()
        self.gridLayout_4.setObjectName(u"gridLayout_4")
        self.roll_pose_label = QLabel(self.layoutWidget)
        self.roll_pose_label.setObjectName(u"roll_pose_label")

        self.gridLayout_4.addWidget(self.roll_pose_label, 0, 0, 1, 1)

        self.roll_pose = QDoubleSpinBox(self.layoutWidget)
        self.roll_pose.setObjectName(u"roll_pose")

        self.gridLayout_4.addWidget(self.roll_pose, 0, 1, 1, 1)


        self.pose_grid_layout.addLayout(self.gridLayout_4, 1, 0, 1, 1)

        self.gridLayout_5 = QGridLayout()
        self.gridLayout_5.setObjectName(u"gridLayout_5")
        self.pitch_pose_label = QLabel(self.layoutWidget)
        self.pitch_pose_label.setObjectName(u"pitch_pose_label")

        self.gridLayout_5.addWidget(self.pitch_pose_label, 0, 0, 1, 1)

        self.pitch_pose = QDoubleSpinBox(self.layoutWidget)
        self.pitch_pose.setObjectName(u"pitch_pose")

        self.gridLayout_5.addWidget(self.pitch_pose, 0, 1, 1, 1)


        self.pose_grid_layout.addLayout(self.gridLayout_5, 1, 1, 1, 1)

        self.gridLayout_6 = QGridLayout()
        self.gridLayout_6.setObjectName(u"gridLayout_6")
        self.yaw_pose_label = QLabel(self.layoutWidget)
        self.yaw_pose_label.setObjectName(u"yaw_pose_label")

        self.gridLayout_6.addWidget(self.yaw_pose_label, 0, 0, 1, 1)

        self.yaw_pose = QDoubleSpinBox(self.layoutWidget)
        self.yaw_pose.setObjectName(u"yaw_pose")

        self.gridLayout_6.addWidget(self.yaw_pose, 0, 1, 1, 1)


        self.pose_grid_layout.addLayout(self.gridLayout_6, 1, 2, 1, 1)

        self.pose_enable = QPushButton(testbuttons)
        self.pose_enable.setObjectName(u"pose_enable")
        self.pose_enable.setGeometry(QRect(209, 70, 91, 22))
        self.twist_enable = QPushButton(testbuttons)
        self.twist_enable.setObjectName(u"twist_enable")
        self.twist_enable.setGeometry(QRect(210, 210, 91, 22))
        self.twist_box = QGroupBox(testbuttons)
        self.twist_box.setObjectName(u"twist_box")
        self.twist_box.setGeometry(QRect(21, 220, 281, 121))
        self.layoutWidget_6 = QWidget(self.twist_box)
        self.layoutWidget_6.setObjectName(u"layoutWidget_6")
        self.layoutWidget_6.setGeometry(QRect(10, 30, 274, 81))
        self.twist_grid_layout = QGridLayout(self.layoutWidget_6)
        self.twist_grid_layout.setSpacing(25)
        self.twist_grid_layout.setObjectName(u"twist_grid_layout")
        self.twist_grid_layout.setContentsMargins(0, 0, 0, 0)
        self.gridLayout_9 = QGridLayout()
        self.gridLayout_9.setObjectName(u"gridLayout_9")
        self.x_twist_label = QLabel(self.layoutWidget_6)
        self.x_twist_label.setObjectName(u"x_twist_label")

        self.gridLayout_9.addWidget(self.x_twist_label, 0, 0, 1, 1)

        self.x_twist = QDoubleSpinBox(self.layoutWidget_6)
        self.x_twist.setObjectName(u"x_twist")

        self.gridLayout_9.addWidget(self.x_twist, 0, 1, 1, 1)


        self.twist_grid_layout.addLayout(self.gridLayout_9, 0, 0, 1, 1)

        self.gridLayout_10 = QGridLayout()
        self.gridLayout_10.setObjectName(u"gridLayout_10")
        self.y_twist_label = QLabel(self.layoutWidget_6)
        self.y_twist_label.setObjectName(u"y_twist_label")

        self.gridLayout_10.addWidget(self.y_twist_label, 0, 0, 1, 1)

        self.y_twist = QDoubleSpinBox(self.layoutWidget_6)
        self.y_twist.setObjectName(u"y_twist")

        self.gridLayout_10.addWidget(self.y_twist, 0, 1, 1, 1)


        self.twist_grid_layout.addLayout(self.gridLayout_10, 0, 1, 1, 1)

        self.gridLayout_11 = QGridLayout()
        self.gridLayout_11.setObjectName(u"gridLayout_11")
        self.z_twist_label = QLabel(self.layoutWidget_6)
        self.z_twist_label.setObjectName(u"z_twist_label")

        self.gridLayout_11.addWidget(self.z_twist_label, 0, 0, 1, 1)

        self.z_twist = QDoubleSpinBox(self.layoutWidget_6)
        self.z_twist.setObjectName(u"z_twist")

        self.gridLayout_11.addWidget(self.z_twist, 0, 1, 1, 1)


        self.twist_grid_layout.addLayout(self.gridLayout_11, 0, 2, 1, 1)

        self.gridLayout_12 = QGridLayout()
        self.gridLayout_12.setObjectName(u"gridLayout_12")
        self.roll_twist_label = QLabel(self.layoutWidget_6)
        self.roll_twist_label.setObjectName(u"roll_twist_label")

        self.gridLayout_12.addWidget(self.roll_twist_label, 0, 0, 1, 1)

        self.roll_twist = QDoubleSpinBox(self.layoutWidget_6)
        self.roll_twist.setObjectName(u"roll_twist")

        self.gridLayout_12.addWidget(self.roll_twist, 0, 1, 1, 1)


        self.twist_grid_layout.addLayout(self.gridLayout_12, 1, 0, 1, 1)

        self.gridLayout_13 = QGridLayout()
        self.gridLayout_13.setObjectName(u"gridLayout_13")
        self.pitch_twist_label = QLabel(self.layoutWidget_6)
        self.pitch_twist_label.setObjectName(u"pitch_twist_label")

        self.gridLayout_13.addWidget(self.pitch_twist_label, 0, 0, 1, 1)

        self.pitch_twist = QDoubleSpinBox(self.layoutWidget_6)
        self.pitch_twist.setObjectName(u"pitch_twist")

        self.gridLayout_13.addWidget(self.pitch_twist, 0, 1, 1, 1)


        self.twist_grid_layout.addLayout(self.gridLayout_13, 1, 1, 1, 1)

        self.gridLayout_14 = QGridLayout()
        self.gridLayout_14.setObjectName(u"gridLayout_14")
        self.yaw_twist_label = QLabel(self.layoutWidget_6)
        self.yaw_twist_label.setObjectName(u"yaw_twist_label")

        self.gridLayout_14.addWidget(self.yaw_twist_label, 0, 0, 1, 1)

        self.yaw_twist = QDoubleSpinBox(self.layoutWidget_6)
        self.yaw_twist.setObjectName(u"yaw_twist")

        self.gridLayout_14.addWidget(self.yaw_twist, 0, 1, 1, 1)


        self.twist_grid_layout.addLayout(self.gridLayout_14, 1, 2, 1, 1)

        self.splitter = QSplitter(testbuttons)
        self.splitter.setObjectName(u"splitter")
        self.splitter.setGeometry(QRect(50, 10, 231, 41))
        self.splitter.setOrientation(Qt.Horizontal)
        self.splitter.setHandleWidth(30)
        self.launch_enable = QPushButton(self.splitter)
        self.launch_enable.setObjectName(u"launch_enable")
        self.splitter.addWidget(self.launch_enable)
        self.controls_enable = QPushButton(self.splitter)
        self.controls_enable.setObjectName(u"controls_enable")
        self.splitter.addWidget(self.controls_enable)

        self.retranslateUi(testbuttons)

        QMetaObject.connectSlotsByName(testbuttons)
    # setupUi

    def retranslateUi(self, testbuttons):
        testbuttons.setWindowTitle(QCoreApplication.translate("testbuttons", u"testbuttons", None))
        self.pose_box.setTitle(QCoreApplication.translate("testbuttons", u"Desired Pose", None))
        self.x_pose_label.setText(QCoreApplication.translate("testbuttons", u"X", None))
        self.y_pose_label.setText(QCoreApplication.translate("testbuttons", u"Y", None))
        self.z_pose_label.setText(QCoreApplication.translate("testbuttons", u"Z", None))
        self.roll_pose_label.setText(QCoreApplication.translate("testbuttons", u"R", None))
        self.pitch_pose_label.setText(QCoreApplication.translate("testbuttons", u"P", None))
        self.yaw_pose_label.setText(QCoreApplication.translate("testbuttons", u"Y", None))
        self.pose_enable.setText(QCoreApplication.translate("testbuttons", u"Start Publishing", None))
        self.twist_enable.setText(QCoreApplication.translate("testbuttons", u"Start Publishing", None))
        self.twist_box.setTitle(QCoreApplication.translate("testbuttons", u"Desired Twist", None))
        self.x_twist_label.setText(QCoreApplication.translate("testbuttons", u"X", None))
        self.y_twist_label.setText(QCoreApplication.translate("testbuttons", u"Y", None))
        self.z_twist_label.setText(QCoreApplication.translate("testbuttons", u"Z", None))
        self.roll_twist_label.setText(QCoreApplication.translate("testbuttons", u"R", None))
        self.pitch_twist_label.setText(QCoreApplication.translate("testbuttons", u"P", None))
        self.yaw_twist_label.setText(QCoreApplication.translate("testbuttons", u"Y", None))
        self.launch_enable.setText(QCoreApplication.translate("testbuttons", u"Run Launch File", None))
        self.controls_enable.setText(QCoreApplication.translate("testbuttons", u"Enable Controls", None))
    # retranslateUi

