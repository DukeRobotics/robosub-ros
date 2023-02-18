from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QAbstractItemView, QTableWidgetItem, QHeaderView
from python_qt_binding.QtCore import QTimer, pyqtProperty
import python_qt_binding.QtCore as QtCore

import rospy
import resource_retriever as rr
import rosservice

from custom_msgs.srv import StopLaunch
from std_msgs.msg import String


class LaunchWidget(QWidget):

    def __init__(self):
        super(LaunchWidget, self).__init__()

        ui_file = rr.get_filename('package://gui/resource/LaunchWidget.ui', use_protocol=False)
        loadUi(ui_file, self)

        self.default_package = ''
        self.pid_rows = {}

        self.table_widget.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.table_widget.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)

        self.launch_dialog.node_launched.connect(self.append_to_table)

        self.remote_launch_timer = QTimer(self)
        self.remote_launch_timer.timeout.connect(self.check_remote_launch)
        self.remote_launch_timer.start(100)

        rospy.Subscriber(self.camera_feed_topic, String, self.check_for_termination)

        rospy.loginfo('Launch Widget successfully initialized')

    @pyqtProperty(str)
    def default_pkg(self):
        return self.default_package

    @default_pkg.setter
    def default_pkg(self, value):
        self.default_package = value
        self.launch_dialog.default_pkg = value

    def keyPressEvent(self, event):
        if event.key() == QtCore.Qt.Key_Delete:
            items = self.table_widget.selectedItems()
            if items and items[0].row() != 0:
                self.delete_launch(items[0].row())
        super(LaunchWidget, self).keyPressEvent(event)

    def check_remote_launch(self):
        enabled = '/start_node' in rosservice.get_service_list()
        self.launch_dialog.setEnabled(enabled)
        self.table_widget.setEnabled(enabled)

    def check_for_termination(self, str_msg):
        msg_type, pid = str_msg.split(" ")[:1]
        if msg_type == "Terminating":
            if self.pid_rows.get(pid) is not None:
                self.table_widget.removeRow(self.pid_rows[pid])

    def delete_launch(self, row_value):
        stop_launch = rospy.ServiceProxy('stop_node', StopLaunch)
        resp = stop_launch(int(self.table_widget.item(row_value, 0).text()))
        self.table_widget.removeRow(row_value)
        return resp.success

    def append_to_table(self, pid, package, name, args):
        self.table_widget.insertRow(self.table_widget.rowCount())
        self.table_widget.setItem(self.table_widget.rowCount() - 1, 0, QTableWidgetItem(str(pid)))
        self.table_widget.setItem(self.table_widget.rowCount() - 1, 1, QTableWidgetItem(package))
        self.table_widget.setItem(self.table_widget.rowCount() - 1, 2, QTableWidgetItem(name))
        self.table_widget.setItem(self.table_widget.rowCount() - 1, 3, QTableWidgetItem(args))
        self.pid_rows[str(pid)] = self.table_widget.rowCount() - 1
        return self.table_widget.rowCount() - 1

    def closeEvent(self, event):
        self.launch_dialog.accept()
