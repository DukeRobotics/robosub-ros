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
    QMessageBox
)
from python_qt_binding.QtCore import QTimer, QObject, QRunnable, QThreadPool, pyqtProperty, pyqtSignal, pyqtSlot
from python_qt_binding.QtGui import QColor

import rospy
import resource_retriever as rr
import rosservice

from custom_msgs.srv import ConnectUSBCamera, ConnectDepthAICamera
from diagnostic_msgs.msg import DiagnosticArray


class CameraStatusDataType(Enum):
    PING = 0
    STEREO = 1
    MONO = 2


CAMERA_STATUS_CAMERA_TYPES = [CameraStatusDataType.STEREO, CameraStatusDataType.MONO]

CAMERA_STATUS_DATA_TYPE_INFORMATION = {
    CameraStatusDataType.PING: {
        'name': 'Ping',
        'index': 0,
        'topic_name': 'ping_ip'
    },
    CameraStatusDataType.STEREO: {
        'name': 'Stereo',
        'index': 1,
        'service_name': '/connect_depthai_camera',
        'service_type': ConnectDepthAICamera
    },
    CameraStatusDataType.MONO: {
        'name': 'Mono',
        'index': 2,
        'service_name': '/connect_usb_camera',
        'service_type': ConnectUSBCamera
    }
}


class CallConnectCameraServiceSignals(QObject):
    connected_signal = pyqtSignal(CameraStatusDataType, bool, str)


class CallConnectCameraService(QRunnable):

    def __init__(self, camera_type):
        super(CallConnectCameraService, self).__init__()

        if camera_type not in CAMERA_STATUS_CAMERA_TYPES:
            raise ValueError('Invalid camera type')

        self.service_name = CAMERA_STATUS_DATA_TYPE_INFORMATION[camera_type]['service_name']
        self.service_type = CAMERA_STATUS_DATA_TYPE_INFORMATION[camera_type]['service_type']
        self.camera_type = camera_type
        self.signals = CallConnectCameraServiceSignals()

    @pyqtSlot()
    def run(self):
        try:
            rospy.wait_for_service(self.service_name, timeout=1)
            connect_camera_service = rospy.ServiceProxy(self.service_name, self.service_type)
            status = connect_camera_service().success
            timestamp = datetime.now().strftime("%H:%M:%S")
            self.signals.connected_signal.emit(self.camera_type, status, timestamp)
        except Exception:
            self.signals.connected_signal.emit(self.camera_type, False, None)


class CameraStatusWidget(QWidget):

    data_updated = pyqtSignal(CameraStatusDataType, bool, str, name='dataUpdated')

    def __init__(self):
        super(CameraStatusWidget, self).__init__()

        ui_file = rr.get_filename('package://gui/resource/CameraStatusWidget.ui', use_protocol=False)
        loadUi(ui_file, self)

        self.ping_hostname = ''
        self.usb_channel = -1

        self.threadpool = QThreadPool()

        self.logs_button.clicked.connect(self.open_conection_log)

        self.check_stereo_button.clicked.connect(lambda: self.check_camera_connection(CameraStatusDataType.STEREO))
        self.check_mono_button.clicked.connect(lambda: self.check_camera_connection(CameraStatusDataType.MONO))

        self.check_camera_buttons = {
            CameraStatusDataType.STEREO: self.check_stereo_button,
            CameraStatusDataType.MONO: self.check_mono_button
        }

        self.checking = {}
        for camera_type in CameraStatusDataType:
            self.checking[camera_type] = False

        self.status_logs = {}
        for camera_type in CameraStatusDataType:
            self.status_logs[camera_type] = []

        self.status_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)

        self.check_buttons_enabled_timer = QTimer(self)
        self.check_buttons_enabled_timer.timeout.connect(self.check_buttons_enabled)
        self.check_buttons_enabled_timer.start(100)

        rospy.Subscriber(
            CAMERA_STATUS_DATA_TYPE_INFORMATION[CameraStatusDataType.PING],
            DiagnosticArray,
            self.ping_response
        )

        self.init_table()

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

    def check_buttons_enabled(self):
        service_list = rosservice.get_service_list()
        for camera_type in self.check_camera_buttons:
            self.check_camera_buttons[camera_type].setEnabled(
                CAMERA_STATUS_DATA_TYPE_INFORMATION[camera_type]['service_name'] in service_list and
                not self.checking[camera_type]
            )

    def open_conection_log(self):
        log = CameraStatusLog(self.data_updated, self.status_logs)
        log.show()

    def check_camera_connection(self, camera_type):
        call_connect_camera_service = CallConnectCameraService(camera_type)
        call_connect_camera_service.signals.connected_signal.connect(self.connected_camera)
        self.threadpool.start(call_connect_camera_service)

        self.checking[camera_type] = True
        self.check_camera_buttons[camera_type].setText("Checking...")

    def connected_camera(self, camera_type, status, timestamp):
        self.checking[camera_type] = False
        self.check_camera_buttons[camera_type].setText(CAMERA_STATUS_DATA_TYPE_INFORMATION[camera_type]['name'])

        if timestamp:
            self.status_logs[camera_type].append({"status": status, "timestamp": timestamp})
            self.data_updated.emit(camera_type, status, timestamp)
            self.update_table(camera_type, status, timestamp)
        else:
            # Display an alert indicating that the service call failed
            alert = QMessageBox()
            alert.setIcon(QMessageBox.Warning)
            alert.setText("Could not complete the service call to connect to the " +
                          f"{CAMERA_STATUS_DATA_TYPE_INFORMATION[camera_type]['name']} camera.")
            alert.exec_()

    def ping_response(self, response):
        # This method is called when a new message is published to the /ping_ip topic

        # TODO: Uncomment once this widget is put into a perspective
        # Make sure response hostname matches self.ping_hostname before proceeding
        # if response.status[0].name != self.ping_hostname:
        #     return

        data_type = CameraStatusDataType.PING
        status = response.status[0].level == 0
        timestamp = datetime.fromtimestamp(response.header.stamp.secs).strftime("%H:%M:%S")

        self.status_logs[data_type].append({"status": status, "timestamp": timestamp})
        self.data_updated.emit(data_type, status, timestamp)
        self.update_table(data_type, status, timestamp)

    def init_table(self):
        for _, data_dict in CAMERA_STATUS_DATA_TYPE_INFORMATION.items():
            self.status_table.insertRow(data_dict["index"])
            self.status_table.setItem(data_dict["index"], 0, QTableWidgetItem(data_dict["name"]))
            self.status_table.setItem(data_dict["index"], 1, QTableWidgetItem("-"))
            self.status_table.setItem(data_dict["index"], 2, QTableWidgetItem("-"))
            self.status_table.setRowHeight(data_dict["index"], 10)

    def update_table(self, type, status, timestamp):
        type_info = CAMERA_STATUS_DATA_TYPE_INFORMATION[type]

        status_msg = "Successful" if status else "Failed"
        color = "green" if status else "red"

        name_item = QTableWidgetItem(type_info["name"])

        status_item = QTableWidgetItem(status_msg)
        status_item.setForeground(QColor(color))

        timestamp_item = QTableWidgetItem(timestamp)

        self.status_table.setItem(type_info["index"], 0, name_item)
        self.status_table.setItem(type_info["index"], 1, status_item)
        self.status_table.setItem(type_info["index"], 2, timestamp_item)


class CameraStatusLog(QDialog):
    def __init__(self, data_updated_signal, init_data):
        super(CameraStatusLog, self).__init__()

        data_updated_signal.connect(self.update)

        layout = QGridLayout()
        tab_widget = QTabWidget()

        self.log_tables = {}
        for data_type in init_data:
            table = QTableWidget()
            table.horizontalHeader().setVisible(False)
            table.verticalHeader().setVisible(False)
            table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
            table.setColumnCount(2)

            tab_widget.addTab(table, CAMERA_STATUS_DATA_TYPE_INFORMATION[data_type]["name"])
            self.log_tables[data_type] = table

        layout.addWidget(tab_widget, 0, 0)
        self.setLayout(layout)

        for type, data in init_data.items():
            for row in data:
                self.update(type, row["status"], row["timestamp"])

    def update(self, type, status, timestamp):
        table = self.log_tables[type]

        status_msg = "Successful" if status else "Failed"
        color = "green" if status else "red"

        status_item = QTableWidgetItem(status_msg)
        status_item.setForeground(QColor(color))

        timestamp_item = QTableWidgetItem(timestamp)

        rowPosition = 0
        table.insertRow(rowPosition)
        table.setItem(rowPosition, 0, status_item)
        table.setItem(rowPosition, 1, timestamp_item)
