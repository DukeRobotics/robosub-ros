from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QMessageBox, QPushButton

import rospy
import resource_retriever as rr

import os


class CameraStatusWidget(QWidget):

    def __init__(self):
        super(CameraStatusWidget, self).__init__()

        ui_file = rr.get_filename('package://gui/resource/CameraStatusWidget.ui', use_protocol=False)
        loadUi(ui_file, self)

        self.check_mono_button.clicked.connect(self.mono_check_connection)
        self.stereo_ping_button.clicked.connect(self.stereo_toggle_ping)
        self.logs_button.clicked.connect(self.open_conection_log)
        self.check_stereo_button.clicked.connect(self.stereo_check_connection)

        self.mono_connection_log = ""
        self.stereo_ping_log = ""
        self.stereo_connection_log = ""
        self.isPinging = False

        self.populate_text_area()

        rospy.loginfo('Camera Status Widget successfully initialized')

    def open_conection_log(self):
        log_message_box = QMessageBox()
        log_message_box.setWindowTitle('Connection Log')
        log_message_box.setText(self.mono_connection_log)

        close_button = QPushButton()
        close_button.setText("Close")
        log_message_box.addButton(close_button, QMessageBox.AcceptRole)

        log_message_box.exec_()

    def mono_check_connection():
        pass

    def stereo_toggle_ping(self):
        button = self.stereo_ping_button

        button.setText("Stop Ping" if self.isPinging == "Start Ping" else "Start Ping")

        hostname = "google.com"
        response = os.system("ping -c 1 " + hostname)

        print(response)

        if not self.isPinging:  # Start ping
            rospy.Service("ping_stereo", EnableModel, self.run_model)
        else:  # End ping
            pass

        self.isPinging = not self.isPinging

        pass

    def stereo_check_connection(self):
        pass

    def populate_mono_camera_text_area(self):
        text_area = self.mono_text_area

        mono_status = {
            "connection": '<span style="color:green">Successful</span>',
            "last_connection": "11:24:67"
        }

        # The first value in each tuple is the title that will be displayed
        # The second value in each tuple is the content corresponding to the title
        contents = [("Connection", mono_status["connection"]),
                    ("Last connection", mono_status["last_connection"])]

        text_area.setText('')

        for content in contents:
            text_area.append('<b>{}</b>: {}'.format(content[0], content[1]))

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
