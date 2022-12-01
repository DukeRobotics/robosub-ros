from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QFileDialog, QDialog, QLineEdit, QPushButton, QInputDialog
from python_qt_binding.QtCore import QTimer, pyqtProperty
# import python_qt_binding.QtCore as QtCore

import rospy
import resource_retriever as rr
import rosservice
import time

from custom_msgs.srv import StartLaunch, StopLaunch
from rqt_bag import topic_selection
from gui.bag_record import BagRecord


class RosbagWidget(QWidget):

    def __init__(self):
        super(RosbagWidget, self).__init__()

        ui_file = rr.get_filename('package://gui/resource/RosbagWidget.ui', use_protocol=False)
        loadUi(ui_file, self)

        self.record_bag_button.clicked.connect(self.click_record)
        self.stop_recording_button.clicked.connect(self.click_stop)

        self.current_launch = None

        self.default_package = ''

        # self.remote_launch_timer = QTimer(self)
        # self.remote_launch_timer.timeout.connect(self.check_remote_launch)
        # self.remote_launch_timer.start(100)

        rospy.loginfo('Rosbag Widget successfully initialized')

    # @pyqtProperty(str)
    # def default_pkg(self):
    #     return self.default_package

    # @default_pkg.setter
    # def default_pkg(self, value):
    #     self.default_package = value
    #     self.launch_dialog.default_pkg = value

    def check_remote_launch(self):
        pass

    def click_record(self):
        self.topic_selection = topic_selection.TopicSelection()
        self.topic_selection.recordSettingsSelected.connect(self._on_record_settings_selected)

    def _on_record_settings_selected(self, all_topics, selected_topics):

        # Get the bag name to record to, prepopulating with a file name based on the current date/time
        proposed_filename = time.strftime('%Y-%m-%d-%H-%M-%S', time.localtime(time.time()))

        filename = QFileDialog.getSaveFileName(
            self, self.tr('Select name for new bag'), proposed_filename, self.tr('Bag files {.bag} (*.bag)'))[0]

        if filename != '':
            filename = filename.strip()
            if not filename.endswith('.bag'):
                filename += ".bag"

            self.bag_record = BagRecord(topics=selected_topics, bag_file_path=filename)
            self.launch_optional_args_dialog()

            # Begin recording
            # self.load_button.setEnabled(False)
            # self._recording = True
            # self._timeline.record_bag(filename, all_topics, selected_topics)

    def click_stop(self):
        # TODO: implement this
        pass

    def launch_optional_args_dialog(self):
        self.optional_args_dialog = QInputDialog()
        self.optional_args_dialog.setOkButtonText("Start Recording")
        optional_args, start_recording = self.optional_args_dialog.getText(self,
            'Optional Arguments',
            'Optionally provide any other arguments to rosbag record:')
        if start_recording:
            self.bag_record.record(optional_args=optional_args)

        # line_edit = QLineEdit(self.optional_args_dialog)
        # cancel_button = QPushButton("Cancel", self.optional_args_dialog)
        # self.optional_args_dialog.setLabelText("Optional Arguments")
        # self.optional_args_dialog.setOkButtonText("Start Recording")

        # optional_args = line_edit.text()
        # start_record_button.clicked.connect(lambda: self.bag_record.record(optional_args))
        # cancel_button.clicked.connect(self.topic_selection.close)

        # self.optional_args_dialog.exec_()

    def populate_text_area(self):
        text_area = self.textArea

        file_name = "file name"
        file_path = "file path"
        topics = "topics"
        other_args = "other args"

        text = f'''
        File Name: 
        {file_name}

        File Path:
        {file_path}

        Topics:
        {topics}

        Other args:
        {other_args}
        '''

        text_area.setReadOnly(True)
        text_area.setText(text)
