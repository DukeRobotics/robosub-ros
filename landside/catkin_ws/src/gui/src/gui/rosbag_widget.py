from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QFileDialog, QInputDialog, QMessageBox
# from python_qt_binding.QtCore import QTimer, pyqtProperty
# import python_qt_binding.QtCore as QtCore

import rospy
import resource_retriever as rr
# import rosservice
import time
import os

# from custom_msgs.srv import StartLaunch, StopLaunch
from rqt_bag import topic_selection
from gui.bag_record import BagRecord


class RosbagWidget(QWidget):

    def __init__(self):
        super(RosbagWidget, self).__init__()

        ui_file = rr.get_filename('package://gui/resource/RosbagWidget.ui', use_protocol=False)
        loadUi(ui_file, self)

        self.record_bag_button.clicked.connect(self.click_record)
        self.stop_recording_button.clicked.connect(self.click_stop)

        self.bag_dict = [{'file name': '', 'file path': '', 'topics': '', 'other args': '', 'BagRecord': ''}]
        self.filenames = ['']
        self.reset(0)

        self.background_colors = {
            "green": "background-color: #8aff92",
            "red": "background-color: #ff7878"
        }
        self.record_bag_button.setStyleSheet(self.background_colors["green"])
        self.stop_recording_button.setStyleSheet(self.background_colors["red"])

        self.bag_files_box.activated.connect(self.bag_files_selected)

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

    def reset(self, index):
        self.bag_files_box.clear()
        self.bag_files_box.addItems(self.filenames)
        self.bag_files_box.setEnabled(True)
        self.stop_recording_button.setEnabled(False)
        self.current_bag_index = index
        self.bag_files_box.setCurrentText(self.filenames[index])
        self.populate_text_area(self.current_bag_index)

    def click_record(self):
        self.topic_selection = topic_selection.TopicSelection()
        self.topic_selection.recordSettingsSelected.connect(self._on_record_settings_selected)

    def _on_record_settings_selected(self, all_topics, selected_topics):

        # Get the bag name to record to, prepopulating with a file name based on the current date/time
        proposed_filename = time.strftime('%Y-%m-%d-%H-%M-%S', time.localtime(time.time()))

        filepath = QFileDialog.getSaveFileName(
            self, self.tr('Select name for new bag'), proposed_filename, self.tr('Bag files {.bag} (*.bag)'))[0]

        if filepath != '':
            filepath = filepath.strip()
            if not filepath.endswith('.bag'):
                filepath += ".bag"

            self.launch_optional_args_dialog(selected_topics, filepath)

            # Begin recording
            # self.load_button.setEnabled(False)
            # self._recording = True
            # self._timeline.record_bag(filename, all_topics, selected_topics)

    def launch_optional_args_dialog(self, selected_topics, filepath):
        optional_args_dialog = QInputDialog()
        optional_args_dialog.setOkButtonText("Start Recording")
        optional_args, start_recording = optional_args_dialog.getText(self, 'Optional Arguments',
            'Optionally provide any other arguments to rosbag record:')

        # Begin recording
        if start_recording:
            bag_record = BagRecord(topics=selected_topics, bag_file_path=filepath)
            bag_record.record(optional_args=optional_args)
            filename = os.path.basename(filepath)
            bag_file = {
                'file name': filename,
                'file path': filepath,  # TODO: retrieve file path
                'topics': selected_topics,
                'other args': optional_args,
                'BagRecord': bag_record
            }
            self.bag_dict.append(bag_file)
            self.filenames.append(filename)
            self.reset(-1)
            self.stop_recording_button.setEnabled(True)

        # line_edit = QLineEdit(self.optional_args_dialog)
        # cancel_button = QPushButton("Cancel", self.optional_args_dialog)
        # self.optional_args_dialog.setLabelText("Optional Arguments")
        # self.optional_args_dialog.setOkButtonText("Start Recording")

        # optional_args = line_edit.text()
        # start_record_button.clicked.connect(lambda: self.bag_record.record(optional_args))
        # cancel_button.clicked.connect(self.topic_selection.close)

        # self.optional_args_dialog.exec_()

    def click_stop(self):
        if self.current_bag_index == 0:
            return

        self.launch_stop_confirmation_dialog()

    def launch_stop_confirmation_dialog(self):
        bag_file = self.bag_dict[self.current_bag_index]

        stop_confirmation = QMessageBox()
        stop_confirmation.setIcon(QMessageBox.Warning)
        stop_confirmation.setWindowTitle('Stop Recording Confirmation')
        stop_confirmation.setText(f'Stop recording {bag_file["file name"]}?')
        stop_confirmation.setStandardButtons(QMessageBox.Cancel | QMessageBox.Yes)
        stop_button = stop_confirmation.button(QMessageBox.Yes)
        stop_button.setText('Stop')
        stop_button.setStyleSheet(self.background_colors["red"])
        stop_confirmation.exec_()

        if stop_confirmation.clickedButton() == stop_button:
            bag_file['BagRecord'].stop()
            self.bag_dict.pop(self.current_bag_index)
            self.filenames.pop(self.current_bag_index)
            self.reset(0)

    def bag_files_selected(self, item_index):
        self.populate_text_area(item_index)
        self.current_bag_index = item_index

        if item_index == 0:
            self.stop_recording_button.setEnabled(False)
        else:
            self.stop_recording_button.setEnabled(True)

    def populate_text_area(self, item_index):
        text_area = self.textArea
        bag_file = self.bag_dict[item_index]
        new_line = '\n'

        filename = bag_file["file name"]
        filepath = bag_file["file path"]
        topics = bag_file["topics"]
        other_args = bag_file["other args"]

        titles = ["File Name", "File Path", "Topics", "Other args"]
        contents = [filename, filepath, topics, other_args]

        text_area.setReadOnly(True)
        text_area.setText('')

        for i in range(len(titles)):
            text_area.append('<b>{}</b>'.format(titles[i]))
            if titles[i] == 'Topics':
                text_area.append(new_line.join(contents[i]))
            else:
                text_area.append(contents[i])
            text_area.append('')
