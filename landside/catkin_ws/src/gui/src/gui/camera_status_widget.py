from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import (
    QWidget,
    QTableWidget,
    QTableWidgetItem,
    QHeaderView,
    QTabWidget,
    QDialog,
    QGridLayout,
)
from python_qt_binding.QtCore import QTimer, pyqtProperty
from python_qt_binding.QtGui import QColor

import rospy
import resource_retriever as rr
import rosservice

from custom_msgs.srv import ConnectUSBCamera, ConnectDepthAICamera
from diagnostic_msgs.msg import DiagnosticArray

from datetime import datetime


class CameraStatusWidget(QWidget):

    def __init__(self):
        super(CameraStatusWidget, self).__init__()

        ui_file = rr.get_filename('package://gui/resource/CameraStatusWidget.ui', use_protocol=False)
        loadUi(ui_file, self)

        self.ping_hostname = ''

        self.logs_button.clicked.connect(self.open_conection_log)
        self.check_mono_button.clicked.connect(self.mono_check_connection)
        self.check_stereo_button.clicked.connect(self.stereo_check_connection)

        self.mono_log_table = QTableWidget()
        self.stereo_log_table = QTableWidget()
        self.ping_log_table = QTableWidget()

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

    def check_camera_test_connection(self):
        self.check_mono_button.setEnabled('/connect_usb_camera' in rosservice.get_service_list())
        self.check_stereo_button.setEnabled('/connect_depthai_camera' in rosservice.get_service_list())

    def open_conection_log(self):
        # TODO: Replace below with QDialog and tabbed widget, with each tab displaying one log table
        # See here for how to get started with tab widget: https://pythonbasics.org/pyqt-tabwidget/

        dialog = QDialog()

        layout = QGridLayout()

        tabwidget = QTabWidget()
        tabwidget.addTab(self.ping_log_table, "Ping")
        tabwidget.addTab(self.stereo_log_table, "Stereo")
        tabwidget.addTab(self.mono_log_table, "Mono")
        layout.addWidget(tabwidget, 0, 0)

        dialog.setLayout(layout)

        dialog.exec_()

    def mono_check_connection(self):
        # Call mono test connection service
        connect_usb_camera = rospy.ServiceProxy('connect_usb_camera', ConnectUSBCamera)
        status = connect_usb_camera(0).success

        # Update mono row in status_table with result
        self.update_table("Mono", status, datetime.now().strftime("%H:%M:%S"))

        # TODO: Add row at top of mono_log_table with result
        self.update_mono_table(status, datetime.now().strftime("%H:%M:%S"))

    def stereo_check_connection(self):
        # Call stereo test connection service
        connect_depthai_camera = rospy.ServiceProxy('connect_depthai_camera', ConnectDepthAICamera)
        status = connect_depthai_camera().success

        # Update stereo row in status_table with result
        self.update_table("Stereo", status, datetime.now().strftime("%H:%M:%S"))

        # TODO: Add row at top of stereo_log_table with result
        self.update_stereo_table(status, datetime.now().strftime("%H:%M:%S"))

    def ping_response(self, response):
        # This method is called when a new message is published to the /ping_ip topic

        # TODO: Uncomment once self.ping_hostname has a value
        # Make sure response hostname matches self.ping_hostname before proceeding
        # if response.status[0].name != self.ping_hostname:
        #     return

        status = response.status[0].level == 0

        timestamp = datetime.fromtimestamp(response.header.stamp.secs).strftime("%H:%M:%S")

        # Update ping row in status_table with result
        # TODO: Format time
        self.update_table("Ping", status, timestamp)

        # TODO: Add row at top of ping_log_table with result
        self.update_ping_table(status, timestamp)

    # SAMPLE ONLY
    def populate_table(self):
        table = self.status_table

        table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)

        ping_status = QTableWidgetItem("Successful")
        ping_status.setForeground(QColor("green"))

        stereo_status = QTableWidgetItem("Successful")
        stereo_status.setForeground(QColor("green"))

        mono_status = QTableWidgetItem("Successful")
        mono_status.setForeground(QColor("green"))

        table.insertRow(0)
        table.setItem(0, 0, QTableWidgetItem("Ping"))
        table.setItem(0, 1, ping_status)
        table.setItem(0, 2, QTableWidgetItem("11:35:47"))
        table.setRowHeight(0, 10)

        table.insertRow(1)
        table.setItem(1, 0, QTableWidgetItem("Stereo"))
        table.setItem(1, 1, stereo_status)
        table.setItem(1, 2, QTableWidgetItem("11:28:49"))
        table.setRowHeight(1, 10)

        table.insertRow(2)
        table.setItem(2, 0, QTableWidgetItem("Mono"))
        table.setItem(2, 1, mono_status)
        table.setItem(2, 2, QTableWidgetItem("11:24:67"))
        table.setRowHeight(2, 10)

    def update_table(self, item, status, timestamp):
        row_dictionary = {
            "Ping": 0,
            "Stereo": 1,
            "Mono": 2
        }
        row = row_dictionary[item]

        status_msg = "Successful" if status else "Failed"
        color = "green" if status else "red"

        status_item = QTableWidgetItem(status_msg)
        status_item.setForeground(QColor(color))

        table = self.status_table
        table.setItem(row, 0, QTableWidgetItem(item))
        table.setItem(row, 1, status_item)
        table.setItem(row, 2, QTableWidgetItem(timestamp))

    def update_mono_table(self, status, timestamp):
        status_msg = "Successful" if status else "Failed"
        color = "green" if status else "red"

        status_item = QTableWidgetItem(status_msg)
        status_item.setForeground(QColor(color))

        table = self.mono_log_table

        table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        table.setColumnCount(2)

        rowPosition = table.rowCount()
        table.insertRow(rowPosition)

        table.setItem(rowPosition, 0, status_item)
        table.setItem(rowPosition, 1, QTableWidgetItem(timestamp))

    def update_stereo_table(self, status, timestamp):
        status_msg = "Successful" if status else "Failed"
        color = "green" if status else "red"

        status_item = QTableWidgetItem(status_msg)
        status_item.setForeground(QColor(color))

        table = self.stereo_log_table

        table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        table.setColumnCount(2)

        rowPosition = table.rowCount()
        table.insertRow(rowPosition)

        table.setItem(rowPosition, 0, status_item)
        table.setItem(rowPosition, 1, QTableWidgetItem(timestamp))

    def update_ping_table(self, status, timestamp):
        status_msg = "Successful" if status else "Failed"
        color = "green" if status else "red"

        status_item = QTableWidgetItem(status_msg)
        status_item.setForeground(QColor(color))

        table = self.ping_log_table

        table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        table.setColumnCount(2)

        rowPosition = 0
        table.insertRow(rowPosition)

        table.setItem(rowPosition, 0, status_item)
        table.setItem(rowPosition, 1, QTableWidgetItem(timestamp))

    # DEPRECATED
    def populate_text_area(self):
        text_area = self.status_text_area

        stereo_status = {
            "ping": '<span style="color:green">Successful</span>',
            "last_ping": "11:35:47",
            "connection": '<span style="color:green">Successful</span>',
            "last_connection": "11:28:49"
        }

        mono_status = {
            "connection": '<span style="color:green">Successful</span>',
            "last_connection": "11:24:67"
        }

        # The first value in each tuple is the title that will be displayed
        # The second value in each tuple is the content corresponding to the title
        stereo_contents = [("Ping", stereo_status["ping"]),
                           ("Last ping", stereo_status["last_ping"]),
                           ("Connection", stereo_status["connection"]),
                           ("Last connection", stereo_status["last_connection"])]
        mono_contents = [("Connection", mono_status["connection"]),
                         ("Last connection", mono_status["last_connection"])]

        text_area.setText('')

        text_area.append('<u>Stereo</u>')
        for content in stereo_contents:
            if content[0] == "Last ping":  # Add new line for clarity
                text_area.append('<b>{}</b>: {} <br>'.format(content[0], content[1]))
            else:
                text_area.append('<b>{}</b>: {}'.format(content[0], content[1]))

        text_area.append('<br> <u>Mono</u>')
        for content in mono_contents:
            text_area.append('<b>{}</b>: {}'.format(content[0], content[1]))