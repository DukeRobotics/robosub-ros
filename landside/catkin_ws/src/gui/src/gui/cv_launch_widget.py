from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QAbstractItemView, QTableWidgetItem
from python_qt_binding.QtCore import QTimer, pyqtProperty
import python_qt_binding.QtCore as QtCore

import rospy
import resource_retriever as rr
import rosservice

from custom_msgs.srv import StartLaunch, StopLaunch


class CVLaunchWidget(QWidget):

    def __init__(self):
        super(CVLaunchWidget, self).__init__()

        ui_file = rr.get_filename('package://gui/resource/CVLaunchWidget.ui', use_protocol=False)
        loadUi(ui_file, self)

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

    def keyPressEvent(self, event):
        if event.key() == QtCore.Qt.Key_Delete:
            items = self.table_widget.selectedItems()
            if items and items[0].row() != 0:
                self.delete_launch(items[0].row())
        super(CVLaunchWidget, self).keyPressEvent(event)

    def check_remote_launch(self):
        enabled = '/start_node' in rosservice.get_service_list()
        self.launch_dialog.setEnabled(enabled)
        self.table_widget.setEnabled(enabled)

    def launch_node_dialog(self):
        self.launch_dialog.reset()

    def launch_file(self, launchfile):
        args = ['sim:=true'] if self.simulation else ['sim:=false']
        start_launch = rospy.ServiceProxy('start_node', StartLaunch)
        resp = start_launch('execute', launchfile, args, True)
        return self.append_to_table(resp.pid, 'execute', launchfile, args[0])

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
        return self.table_widget.rowCount() - 1

    def closeEvent(self, event):
        self.launch_dialog.accept()
