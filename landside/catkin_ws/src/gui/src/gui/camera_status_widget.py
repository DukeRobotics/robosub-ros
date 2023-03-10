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
    QMessageBox,
    QAbstractItemView,
    QLineEdit,
    QDialogButtonBox,
    QFormLayout,
    QCheckBox,
    QLabel
)
from python_qt_binding.QtCore import QTimer, QObject, QRunnable, QThreadPool, pyqtProperty, pyqtSignal, pyqtSlot
from python_qt_binding.QtGui import QColor, QIntValidator

import rospy
import rosgraph
import rostopic
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
        'topic_name': '/ping_host'
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

    def __init__(self, camera_type, service_args):
        super(CallConnectCameraService, self).__init__()

        if camera_type not in CAMERA_STATUS_CAMERA_TYPES:
            raise ValueError('Invalid camera type')

        self.service_name = CAMERA_STATUS_DATA_TYPE_INFORMATION[camera_type]['service_name']
        self.service_type = CAMERA_STATUS_DATA_TYPE_INFORMATION[camera_type]['service_type']
        self.camera_type = camera_type
        self.signals = CallConnectCameraServiceSignals()
        self.service_args = service_args

    @pyqtSlot()
    def run(self):
        try:
            rospy.wait_for_service(self.service_name, timeout=1)
            connect_camera_service = rospy.ServiceProxy(self.service_name, self.service_type)
            status = connect_camera_service(*self.service_args).success
            timestamp = datetime.now().strftime("%H:%M:%S")
            self.signals.connected_signal.emit(self.camera_type, status, timestamp)
        except Exception:
            self.signals.connected_signal.emit(self.camera_type, False, None)


class CameraStatusWidget(QWidget):

    data_updated = pyqtSignal(CameraStatusDataType, bool, str, str, name='data_updated')

    def __init__(self):
        super(CameraStatusWidget, self).__init__()

        ui_file = rr.get_filename('package://gui/resource/CameraStatusWidget.ui', use_protocol=False)
        loadUi(ui_file, self)

        self.ping_hostname = ''
        self.usb_channel = -1

        self.log = None

        self.threadpool = QThreadPool()

        self.logs_button.clicked.connect(self.open_conection_log)

        self.check_camera_buttons = {
            CameraStatusDataType.STEREO: self.check_stereo_button,
            CameraStatusDataType.MONO: self.check_mono_button
        }

        for camera_type in self.check_camera_buttons:
            self.check_camera_buttons[camera_type].clicked.connect(
                lambda _, camera_type=camera_type: self.check_camera_connection(camera_type)
            )

        self.camera_service_args = {
            CameraStatusDataType.STEREO: lambda: (),
            CameraStatusDataType.MONO: lambda: (self.channel,)
        }

        self.checking = {}
        for camera_type in CAMERA_STATUS_CAMERA_TYPES:
            self.checking[camera_type] = False

        self.status_logs = {}
        for data_type in CameraStatusDataType:
            self.status_logs[data_type] = []

        self.status_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.timer_check)
        self.timer.start(100)

        self.subscriber = rospy.Subscriber(
            CAMERA_STATUS_DATA_TYPE_INFORMATION[CameraStatusDataType.PING]["topic_name"],
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

    def timer_check(self):
        self.check_buttons_enabled()
        self.check_ping_publisher()

    def check_ping_publisher(self):
        master = rosgraph.Master('/rostopic')
        pubs, _ = rostopic.get_topic_list(master=master)
        for topic_name, _, publishing_nodes in pubs:
            if topic_name == CAMERA_STATUS_DATA_TYPE_INFORMATION[CameraStatusDataType.PING]["topic_name"] and \
                    len(publishing_nodes) > 0:
                if self.subscriber is None:
                    self.create_new_subscriber()
                return

        if self.subscriber is not None:
            self.remove_subscriber()

    def create_new_subscriber(self):
        self.subscriber = rospy.Subscriber(
            CAMERA_STATUS_DATA_TYPE_INFORMATION[CameraStatusDataType.PING]["topic_name"],
            DiagnosticArray,
            self.ping_response
        )

    def remove_subscriber(self):
        self.subscriber.unregister()
        self.subscriber = None

    def check_buttons_enabled(self):
        service_list = rosservice.get_service_list()
        for camera_type in self.check_camera_buttons:
            self.check_camera_buttons[camera_type].setEnabled(
                CAMERA_STATUS_DATA_TYPE_INFORMATION[camera_type]['service_name'] in service_list and
                not self.checking[camera_type]
            )

    def open_conection_log(self):
        self.log = CameraStatusLog(self.data_updated, self.status_logs)
        self.log.exec()

    def check_camera_connection(self, camera_type):
        call_connect_camera_service = CallConnectCameraService(camera_type, self.camera_service_args[camera_type]())
        call_connect_camera_service.signals.connected_signal.connect(self.connected_camera)
        self.threadpool.start(call_connect_camera_service)

        self.checking[camera_type] = True
        self.check_camera_buttons[camera_type].setText("Checking...")

    def connected_camera(self, camera_type, status, timestamp):
        self.checking[camera_type] = False
        self.check_camera_buttons[camera_type].setText(CAMERA_STATUS_DATA_TYPE_INFORMATION[camera_type]['name'])

        if timestamp:
            self.status_logs[camera_type].append({"status": status, "timestamp": timestamp, "message": None})
            self.data_updated.emit(camera_type, status, timestamp, None)
            self.update_table(camera_type, status, timestamp)
        else:
            # Display an alert indicating that the service call failed
            alert = QMessageBox()
            alert.setIcon(QMessageBox.Warning)
            alert.setText("Could not complete the service call to connect to the " +
                          f"{CAMERA_STATUS_DATA_TYPE_INFORMATION[camera_type]['name']} camera.")
            alert.exec_()

    def ping_response(self, response):
        # This method is called when a new message is published to the ping topic

        # Make sure response hostname matches self.ping_hostname before proceeding
        if response.status[0].name != self.ping_hostname:
            return

        data_type = CameraStatusDataType.PING
        status_info = {}
        status_info["status"] = response.status[0].level == 0
        status_info["message"] = response.status[0].message
        status_info["timestamp"] = datetime.fromtimestamp(response.header.stamp.secs).strftime("%H:%M:%S")

        self.status_logs[data_type].append(status_info)
        self.data_updated.emit(data_type, status_info["status"], status_info["timestamp"], status_info["message"])
        self.update_table(data_type, status_info["status"], status_info["timestamp"])

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

    def help(self):
        text = "This widget allows you to check the status of the cameras on the robot.\n\n" + \
            "To check if the stereo camera can be pinged, launch cv/ping_host.launch. This plugin will only " + \
            f"display the ping status for {self.ping_hostname}.\n\n" + \
            "To check if the mono and stereo cameras are connected, launch cv/camera_test_connect.launch and click " + \
            "the 'Mono' and 'Stereo' buttons. If camera_test_connect.launch is not running, the buttons will be " + \
            f"disabled. The channel used for the mono camera is {self.usb_channel}.\n\n" + \
            "To change the ping hostname or mono camera channel, click the settings icon. If the plugin appears to " + \
            "be unresponsive to publishing ping messages, you can restart the ping subscriber from settings."

        alert = QMessageBox()
        alert.setWindowTitle("Camera Status Widget Help")
        alert.setIcon(QMessageBox.Information)
        alert.setText(text)
        alert.exec_()

    def settings(self):
        settings = CameraStatusWidgetSettings(self, self.ping_hostname, self.usb_channel)
        if settings.exec_():
            self.hostname, self.channel, restart_ping = settings.get_values()
            if restart_ping and self.subscriber is not None:
                self.remove_subscriber()
                self.create_new_subscriber()

    def close(self):
        self.timer.stop()

        if self.log and self.log.isVisible():
            self.log.close()

        if self.subscriber is not None:
            self.remove_subscriber()

        self.threadpool.clear()
        if self.threadpool.activeThreadCount() > 0:
            message = f"Camera Status Widget waiting for {self.threadpool.activeThreadCount()} threads to finish. " + \
                      "It will close automatically when all threads are finished."
            rospy.loginfo(message)

            alert = QMessageBox()
            alert.setWindowTitle("Waiting for Threads to Finish")
            alert.setIcon(QMessageBox.Information)
            alert.setText(message)
            alert.exec_()

            self.threadpool.waitForDone()

            rospy.loginfo("Camera Status Widget has finished all threads and is successfully closed.")


class CameraStatusLog(QDialog):
    def __init__(self, data_updated_signal, init_data):
        super(CameraStatusLog, self).__init__()

        data_updated_signal.connect(self.update)

        layout = QGridLayout()
        tab_widget = QTabWidget()

        self.data = {}

        self.log_tables = {}
        for data_type in init_data:
            self.data[data_type] = []

            table = QTableWidget()
            table.horizontalHeader().setVisible(False)
            table.verticalHeader().setVisible(False)
            table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
            table.setColumnCount(2)
            table.setEditTriggers(QAbstractItemView.NoEditTriggers)
            if data_type == CameraStatusDataType.PING:
                table.cellDoubleClicked.connect(self.table_clicked)

            tab_widget.addTab(table, CAMERA_STATUS_DATA_TYPE_INFORMATION[data_type]["name"])
            self.log_tables[data_type] = table

        layout.addWidget(tab_widget, 0, 0)
        self.setLayout(layout)

        for type, data in init_data.items():
            for row in data:
                self.update(type, row["status"], row["timestamp"], row["message"])

    def update(self, type, status, timestamp, message):
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

        self.data[type].insert(0, {"status": status, "timestamp": timestamp, "message": message})

    def table_clicked(self, index):
        message = self.data[CameraStatusDataType.PING][index]["message"]

        alert = QMessageBox()
        alert.setWindowTitle("Ping Message")
        alert.setIcon(QMessageBox.Information)
        alert.setText(message)
        alert.exec_()


class CameraStatusWidgetSettings(QDialog):
    def __init__(self, parent, ping_hostname, usb_channel):
        super().__init__(parent)

        self.given_usb_channel = usb_channel

        self.ping_hostname_line_edit = QLineEdit(self)
        self.ping_hostname_line_edit.setText(str(ping_hostname))

        self.usb_channel_line_edit = QLineEdit(self)
        self.usb_channel_line_edit.setText(str(usb_channel))

        validator = QIntValidator(self)
        validator.setBottom(0)
        self.usb_channel_line_edit.setValidator(validator)

        self.restart_ping_subscriber_checkbox = QCheckBox(self)
        self.restart_ping_subscriber_label = QLabel("Restart Ping Subscriber (?)", self)
        self.restart_ping_subscriber_label.setToolTip("If checked, the ping subscriber will be restarted when the " +
                                                      "settings are saved. This is useful if the ping hostname has " +
                                                      "changed, ping_host.launch has been recently restarted, or " +
                                                      "if the plugin does not appear to receive ping messages even" +
                                                      "though ping_host.launch is running.")

        buttonBox = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel, self)

        layout = QFormLayout(self)
        layout.addRow("Ping Hostname", self.ping_hostname_line_edit)
        layout.addRow("USB Channel", self.usb_channel_line_edit)
        layout.addRow(self.restart_ping_subscriber_label, self.restart_ping_subscriber_checkbox)
        layout.addWidget(buttonBox)

        buttonBox.accepted.connect(self.accept)
        buttonBox.rejected.connect(self.reject)

    def get_values(self):
        channel = self.usb_channel_line_edit.text()
        try:
            channel = int(channel)
        except Exception:
            rospy.logwarn("Invalid USB channel (not an integer). The USB channel has not been changed.")
            channel = self.given_usb_channel

        return (self.ping_hostname_line_edit.text(), channel, self.restart_ping_subscriber_checkbox.isChecked())
