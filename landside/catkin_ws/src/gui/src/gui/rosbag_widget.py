from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QAbstractItemView, QTableWidgetItem
from python_qt_binding.QtCore import QTimer, pyqtProperty
# import python_qt_binding.QtCore as QtCore

import rospy
import resource_retriever as rr
import rosservice

from custom_msgs.srv import StartLaunch, StopLaunch
from rqt_bag import topic_selection


class RosbagWidget(QWidget):

    def __init__(self):
        super(RosbagWidget, self).__init__()

        ui_file = rr.get_filename('package://gui/resource/RosbagWidget.ui', use_protocol=False)
        loadUi(ui_file, self)

        self.record_bag_button.clicked.connect(self.click_record)
        self.stop_recording_button.clicked.connect(self.click_stop)

        self.current_launch = None

        self.default_package = ''

        self.table_widget.setSelectionBehavior(QAbstractItemView.SelectRows)

        self.launch_dialog.node_launched.connect(self.append_to_table)

        self.remote_launch_timer = QTimer(self)
        self.remote_launch_timer.timeout.connect(self.check_remote_launch)
        self.remote_launch_timer.start(100)

        rospy.loginfo('CV Launch Widget successfully initialized')

    @pyqtProperty(str)
    def default_pkg(self):
        return self.default_package

    @default_pkg.setter
    def default_pkg(self, value):
        self.default_package = value
        self.launch_dialog.default_pkg = value

    def click_record(self):
        pass

    def click_stop(self):
        pass
