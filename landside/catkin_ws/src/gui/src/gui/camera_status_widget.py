from datetime import datetime
from enum import Enum

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import (
    QWidget,
    QTableWidget,
    QHeaderView,
    QTableWidgetItem,
    QTabWidget,
    QDialog,
    QGridLayout,
)
from python_qt_binding.QtCore import QTimer, pyqtProperty, pyqtSignal
from python_qt_binding.QtGui import QColor

import rospy
import resource_retriever as rr
import rosservice

from custom_msgs.srv import ConnectUSBCamera, ConnectDepthAICamera
from diagnostic_msgs.msg import DiagnosticArray


class CameraStatusDataUpdateType(Enum):
    PING = 1
    STEREO = 2
    MONO = 3


class CameraStatusWidget(QWidget):

    data_updated = pyqtSignal(CameraStatusDataUpdateType, bool, str, name='dataUpdated')

    def __init__(self):
        super(CameraStatusWidget, self).__init__()

        ui_file = rr.get_filename('package://gui/resource/CameraStatusWidget.ui', use_protocol=False)
        loadUi(ui_file, self)

        self.ping_hostname = ''
        self.usb_channel = -1

        self.logs_button.clicked.connect(self.open_conection_log)
        self.check_mono_button.clicked.connect(self.mono_check_connection)
        self.check_stereo_button.clicked.connect(self.stereo_check_connection)

        self.mono_log = []
        self.stereo_log = []
        self.ping_log = []

        self.remote_camera_test_connection_timer = QTimer(self)
        self.remote_camera_test_connection_timer.timeout.connect(self.check_camera_test_connection)
        self.remote_camera_test_connection_timer.start(100)

        rospy.Subscriber('/ping_ip', DiagnosticArray, self.ping_response)

        self.populate_table()

        rospy.loginfo('Camera Status Widget successfully initialized')

    @pyqtProperty(str)
    def hostname(self):
        return self.ping_hostname

    @hostname.setter
    def hostname(self, value):
        self.ping_hostname = value

    @pyqtProperty(int)
    def channel(self):
        return self.usb_channel

    @channel.setter
    def channel(self, value):
        self.usb_channel = value

    def check_camera_test_connection(self):
        self.check_mono_button.setEnabled('/connect_usb_camera' in rosservice.get_service_list())
        self.check_stereo_button.setEnabled('/connect_depthai_camera' in rosservice.get_service_list())

    def open_conection_log(self):
        init_data = {
            CameraStatusDataUpdateType.PING: self.ping_log,
            CameraStatusDataUpdateType.STEREO: self.stereo_log,
            CameraStatusDataUpdateType.MONO: self.mono_log
        }
        log = CameraStatusLog(self.data_updated, init_data)
        log.exec_()

    def mono_check_connection(self):
        # Call mono test connection service
        connect_usb_camera = rospy.ServiceProxy('connect_usb_camera', ConnectUSBCamera)

        # TODO: Uncomment once this widget is put into a perspective
        # status = connect_usb_camera(self.usb_channel).success

        status = connect_usb_camera(0).success

        timestamp = datetime.now().strftime("%H:%M:%S")

        self.mono_log.insert(0, {"status": status, "timestamp": timestamp})

        self.data_updated.emit(CameraStatusDataUpdateType.MONO, status, timestamp)

        # Update mono row in status_table with result
        self.update_table(CameraStatusDataUpdateType.MONO, status, timestamp)

    def stereo_check_connection(self):
        # Call stereo test connection service
        connect_depthai_camera = rospy.ServiceProxy('connect_depthai_camera', ConnectDepthAICamera)

        status = connect_depthai_camera().success
        timestamp = datetime.now().strftime("%H:%M:%S")

        self.stereo_log.insert(0, {"status": status, "timestamp": timestamp})

        self.data_updated.emit(CameraStatusDataUpdateType.STEREO, status, timestamp)

        # Update stereo row in status_table with result
        self.update_table(CameraStatusDataUpdateType.STEREO, status, timestamp)

    def ping_response(self, response):
        # This method is called when a new message is published to the /ping_ip topic

        # TODO: Uncomment once this widget is put into a perspective
        # Make sure response hostname matches self.ping_hostname before proceeding
        # if response.status[0].name != self.ping_hostname:
        #     return

        status = response.status[0].level == 0
        timestamp = datetime.fromtimestamp(response.header.stamp.secs).strftime("%H:%M:%S")

        self.ping_log.insert(0, {"status": status, "timestamp": timestamp})

        self.data_updated.emit(CameraStatusDataUpdateType.PING, status, timestamp)

        # Update ping row in status_table with result
        self.update_table(CameraStatusDataUpdateType.PING, status, timestamp)

    def populate_table(self):
        table = self.status_table

        table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)

        row_names = ["Ping", "Stereo", "Mono"]
        for index, row_name in enumerate(row_names):
            table.insertRow(index)
            table.setItem(index, 0, QTableWidgetItem(row_name))
            table.setItem(index, 1, QTableWidgetItem("-"))
            table.setItem(index, 2, QTableWidgetItem("-"))
            table.setRowHeight(index, 10)

    def update_table(self, type, status, timestamp):
        type_dictionary = {
            CameraStatusDataUpdateType.PING: {"index": 0, "name": "Ping"},
            CameraStatusDataUpdateType.STEREO: {"index": 1, "name": "Stereo"},
            CameraStatusDataUpdateType.MONO: {"index": 2, "name": "Mono"}
        }
        type_info = type_dictionary[type]

        status_msg = "Successful" if status else "Failed"
        color = "green" if status else "red"

        status_item = QTableWidgetItem(status_msg)
        status_item.setForeground(QColor(color))

        table = self.status_table
        table.setItem(type_info["index"], 0, QTableWidgetItem(type_info["name"]))
        table.setItem(type_info["index"], 1, status_item)
        table.setItem(type_info["index"], 2, QTableWidgetItem(timestamp))


class CameraStatusLog(QDialog):
    def __init__(self, data_updated_signal, init_data):
        super(CameraStatusLog, self).__init__()

        self.mono_log_table = QTableWidget()
        self.stereo_log_table = QTableWidget()
        self.ping_log_table = QTableWidget()

        self.mono_log_table.horizontalHeader().setVisible(False)
        self.mono_log_table.verticalHeader().setVisible(False)

        self.stereo_log_table.horizontalHeader().setVisible(False)
        self.stereo_log_table.verticalHeader().setVisible(False)

        self.ping_log_table.horizontalHeader().setVisible(False)
        self.ping_log_table.verticalHeader().setVisible(False)

        data_updated_signal.connect(self.update)

        layout = QGridLayout()

        tabwidget = QTabWidget()
        tabwidget.addTab(self.ping_log_table, "Ping")
        tabwidget.addTab(self.stereo_log_table, "Stereo")
        tabwidget.addTab(self.mono_log_table, "Mono")
        layout.addWidget(tabwidget, 0, 0)

        self.setLayout(layout)

        for type, data in init_data.items():
            for row in data:
                self.update(type, row["status"], row["timestamp"])

    def update(self, type, status, timestamp):
        if type == CameraStatusDataUpdateType.PING:
            table = self.ping_log_table
        elif type == CameraStatusDataUpdateType.STEREO:
            table = self.stereo_log_table
        elif type == CameraStatusDataUpdateType.MONO:
            table = self.mono_log_table

        status_msg = "Successful" if status else "Failed"
        color = "green" if status else "red"

        status_item = QTableWidgetItem(status_msg)
        status_item.setForeground(QColor(color))

        table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        table.setColumnCount(2)

        rowPosition = 0
        table.insertRow(rowPosition)

        table.setItem(rowPosition, 0, status_item)
        table.setItem(rowPosition, 1, QTableWidgetItem(timestamp))
