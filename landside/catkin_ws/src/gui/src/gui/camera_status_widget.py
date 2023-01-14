from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QMessageBox, QPushButton

import rospy
import resource_retriever as rr


class CameraStatusWidget(QWidget):

    def __init__(self):
        super(CameraStatusWidget, self).__init__()

        ui_file = rr.get_filename('package://gui/resource/CameraStatusWidget.ui', use_protocol=False)
        loadUi(ui_file, self)

        self.mono_connection_log_button.clicked.connect(self.mono_open_conection_log)
        self.mono_check_connection_button.clicked.connect(self.mono_check_connection)

        self.stereo_ping_log_button.clicked.connect(self.stereo_open_ping_log)
        self.stereo_ping_button.clicked.connect(self.stereo_start_ping)
        self.stereo_connection_log_button.clicked.connect(self.stereo_open_conection_log)
        self.stereo_check_connection_button.clicked.connect(self.stereo_check_connection)

        self.init_gui()

        rospy.loginfo('Camera Status Widget successfully initialized')

    def init_gui(self):
        self.populate_mono_camera_text_area()
        self.populate_stereo_camera_text_area()

    def mono_open_conection_log(self):
        log_message_box = QMessageBox()
        log_message_box.setWindowTitle('Mono Connection Log')
        log_message_box.setText("Lorem ipsum dolor sit amet. " * 20)

        close_button = QPushButton()
        close_button.setText("Close")
        log_message_box.addButton(close_button, QMessageBox.AcceptRole)

        log_message_box.exec_()

    def stereo_open_ping_log(self):
        log_message_box = QMessageBox()
        log_message_box.setWindowTitle('Stereo Ping Log')
        log_message_box.setText("Lorem ipsum dolor sit amet. " * 20)

        close_button = QPushButton()
        close_button.setText("Close")
        log_message_box.addButton(close_button, QMessageBox.AcceptRole)

        log_message_box.exec_()

    def stereo_open_conection_log(self):
        log_message_box = QMessageBox()
        log_message_box.setWindowTitle('Stereo Connection Log')
        log_message_box.setText("Lorem ipsum dolor sit amet. " * 20)

        close_button = QPushButton()
        close_button.setText("Close")
        log_message_box.addButton(close_button, QMessageBox.AcceptRole)

        log_message_box.exec_()
        
    def mono_check_connection():
        pass

    def stereo_start_ping(self):
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

    def populate_stereo_camera_text_area(self):
        text_area = self.stereo_text_area

        mono_status = {
            "ping": '<span style="color:green">Successful</span>',
            "last_ping": "11:35:47",
            "connection": '<span style="color:green">Successful</span>',
            "last_connection": "11:28:49"
        }

        # The first value in each tuple is the title that will be displayed
        # The second value in each tuple is the content corresponding to the title
        contents = [("Ping", mono_status["ping"]),
                    ("Last ping", mono_status["last_ping"]),
                    ("Connection", mono_status["connection"]),
                    ("Last connection", mono_status["last_connection"])]

        text_area.setText('')

        for content in contents:
            if content[0] == "Last ping":  # Add new line for clarity
                text_area.append('<b>{}</b>: {} <br>'.format(content[0], content[1]))
            else:
                text_area.append('<b>{}</b>: {}'.format(content[0], content[1]))
