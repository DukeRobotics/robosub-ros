from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QFileDialog, QInputDialog, QMessageBox, QPushButton

import rospy
import resource_retriever as rr

import time
import os

from rqt_bag import topic_selection
from gui.bag_record import BagRecord


class RosbagWidget(QWidget):

    def __init__(self):
        super(RosbagWidget, self).__init__()

        ui_file = rr.get_filename('package://gui/resource/RosbagWidget.ui', use_protocol=False)
        loadUi(ui_file, self)

        self.record_bag_button.clicked.connect(self.click_record)
        self.active_bag_box.activated.connect(self.bag_files_selected)
        self.stop_recording_button.clicked.connect(self.click_stop)

        self.bag_dict = []
        self.filenames = []
        self.reset(0)

        self.background_colors = {
            "green": "background-color: #8aff92",
            "red": "background-color: #ff7878"
        }
        self.record_bag_button.setStyleSheet(self.background_colors["green"])
        self.stop_recording_button.setStyleSheet(self.background_colors["red"])

        rospy.loginfo('Rosbag Widget successfully initialized')

    def reset(self, index):
        self.active_bag_box.clear()

        if len(self.bag_dict) == 0:
            self.active_bag_box.setEnabled(False)
            self.stop_recording_button.setEnabled(False)
        else:
            self.current_bag_index = index
            self.active_bag_box.addItems(self.filenames)
            self.active_bag_box.setEnabled(True)
            self.active_bag_box.setCurrentText(self.filenames[index])
            self.stop_recording_button.setEnabled(True)

        self.populate_text_area(index)

    def click_record(self):
        self.topic_selection = topic_selection.TopicSelection()
        self.topic_selection.recordSettingsSelected.connect(self.on_record_settings_selected)

    def on_record_settings_selected(self, all_topics, selected_topics):

        # Get the bag name to record to, prepopulating with a file name based on the current date/time
        proposed_filename = "/root/dev/robosub-ros/" + time.strftime('%Y-%m-%d-%H-%M-%S', time.localtime(time.time()))

        filepath = QFileDialog.getSaveFileName(
            self, self.tr('Select name for new bag'), proposed_filename, self.tr('Bag files {.bag} (*.bag)'))[0]

        if filepath != '':
            filepath = filepath.strip()
            if not filepath.endswith('.bag'):
                filepath += ".bag"

            self.launch_optional_args_dialog(selected_topics, filepath)

    def launch_optional_args_dialog(self, selected_topics, filepath):
        optional_args_dialog = QInputDialog()
        optional_args_dialog.setOkButtonText("Start Recording")
        optional_args, start_recording = optional_args_dialog.getText(self, 'Optional Arguments', ('Optionally provide '
                                                                      'any other arguments to rosbag record:'))

        # Begin recording
        if start_recording:
            bag_record = BagRecord(topics=selected_topics, bag_file_path=filepath)
            bag_record.record(optional_args=optional_args)
            filename = os.path.basename(filepath)
            bag_file = {
                'filename': filename,
                'filepath': filepath,
                'topics': selected_topics,
                'other_args': optional_args,
                'bag_record': bag_record
            }
            self.bag_dict.append(bag_file)
            self.filenames.append(filename)
            self.reset(-1)
            self.stop_recording_button.setEnabled(True)

    def click_stop(self):
        self.launch_stop_confirmation_dialog()

    def launch_stop_confirmation_dialog(self):
        bag_file = self.bag_dict[self.current_bag_index]

        stop_confirmation = QMessageBox()
        stop_confirmation.setIcon(QMessageBox.Warning)
        stop_confirmation.setWindowTitle('Confirm Stop')
        stop_confirmation.setText(f'Stop recording {bag_file["filename"]}?')

        cancel_button = QPushButton()
        cancel_button.setText("Cancel")
        stop_confirmation.addButton(cancel_button, QMessageBox.RejectRole)

        stop_button = QPushButton()
        stop_button.setText('Stop')
        stop_button.setStyleSheet(self.background_colors["red"])
        stop_confirmation.addButton(stop_button, QMessageBox.DestructiveRole)

        stop_confirmation.exec_()

        if stop_confirmation.clickedButton() == stop_button:
            bag_file['bag_record'].stop()
            self.bag_dict.pop(self.current_bag_index)
            self.filenames.pop(self.current_bag_index)
            self.reset(0)

    def bag_files_selected(self, item_index):
        self.populate_text_area(item_index)
        self.current_bag_index = item_index
        self.stop_recording_button.setEnabled(True)

    def populate_text_area(self, item_index):
        text_area = self.bag_information_text_area

        if len(self.bag_dict) == 0:
            text_area.setEnabled(False)
            text_area.setText('<i>No bag files being actively recorded.</i>')
            return

        text_area.setEnabled(True)

        bag_file = self.bag_dict[item_index]
        new_line = '\n'

        # The first value in each tuple is the title that will be displayed
        # The second value in each tuple is the content corresponding to the title
        contents = [("File Name", bag_file["filename"]), ("File Path", bag_file["filepath"]),
                    ("Topics", bag_file["topics"]), ("Other Args", bag_file["other_args"])]

        text_area.setText('')

        for content in contents:
            text_area.append('<b>{}</b>'.format(content[0]))
            if not content[1]:
                text_area.append('<i>None</i>')
            elif content[0] == 'Topics':
                text_area.append(new_line.join(content[1]))
            else:
                text_area.append(content[1])
            text_area.append('')

    def closeWidget(self):
        for bag in self.bag_dict:
            bag['bag_record'].stop()
