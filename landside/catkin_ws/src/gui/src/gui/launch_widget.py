from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import (
    QWidget,
    QAbstractItemView,
    QTableWidgetItem,
    QHeaderView,
    QDialog,
    QCheckBox,
    QLabel,
    QDialogButtonBox,
    QFormLayout,
    QTableWidget,
    QVBoxLayout
)
from python_qt_binding.QtCore import QTimer, pyqtProperty
import python_qt_binding.QtCore as QtCore

import rospy
import resource_retriever as rr
import rosservice

from custom_msgs.srv import StopLaunch
from custom_msgs.msg import RemoteLaunchInfo

from threading import Lock


class LaunchWidget(QWidget):

    def __init__(self):
        super(LaunchWidget, self).__init__()

        ui_file = rr.get_filename('package://gui/resource/LaunchWidget.ui', use_protocol=False)
        loadUi(ui_file, self)

        self.running_nodes = {}

        self.display_all_nodes = False

        self.default_package = ''
        self.table_widget_lock = Lock()

        self.table_widget.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.table_widget.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self.table_widget.cellDoubleClicked.connect(self.row_double_clicked)

        self.launch_dialog.node_launched.connect(self.append_to_table)

        self.remote_launch_timer = QTimer(self)
        self.remote_launch_timer.timeout.connect(self.check_remote_launch)
        self.remote_launch_timer.start(100)

        rospy.Subscriber('remote_launch', RemoteLaunchInfo, self.check_for_termination)
        rospy.Subscriber('remote_launch', RemoteLaunchInfo, self.check_for_new_nodes)

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
                self.delete_launch(items[0].text())
        super(LaunchWidget, self).keyPressEvent(event)

    def check_remote_launch(self):
        enabled = '/start_node' in rosservice.get_service_list()
        self.launch_dialog.setEnabled(enabled)
        self.table_widget.setEnabled(enabled)

    def check_for_termination(self, rli_msg):
        if rli_msg.msg_type == "Terminating":
            self.remove_from_table(rli_msg.pid)

    def check_for_new_nodes(self, rli_msg):
        if self.display_all_nodes:
            if rli_msg.msg_type.startswith("Executing"):
                # If the new node was launched from another plugin instance
                self.append_to_table(rli_msg.pid, rli_msg.package, rli_msg.file, " ".join(rli_msg.args))

    def delete_launch(self, pid):
        stop_launch = rospy.ServiceProxy('stop_node', StopLaunch)
        resp = stop_launch(int(pid))
        self.remove_from_table(pid)
        return resp.success

    def get_row_with_pid(self, pid):
        # Any function must acquire the table_widget_lock before calling this function
        for row in range(self.table_widget.rowCount()):
            if self.table_widget.item(row, 0).text() == str(pid):
                return row
        return None

    def remove_from_table(self, pid):
        self.table_widget_lock.acquire()
        row = self.get_row_with_pid(pid)
        if row is not None:
            self.table_widget.removeRow(row)
            self.running_nodes.pop(str(pid))
        self.table_widget_lock.release()

    def append_to_table(self, pid, package, name, args):
        self.table_widget_lock.acquire()
        if pid not in self.running_nodes:
            self.running_nodes[str(pid)] = {
                "PID": str(pid),
                "Package": package,
                "File": name,
                "Args": args
            }

            self.table_widget.insertRow(self.table_widget.rowCount())
            self.table_widget.setItem(self.table_widget.rowCount() - 1, 0, QTableWidgetItem(str(pid)))
            self.table_widget.setItem(self.table_widget.rowCount() - 1, 1, QTableWidgetItem(package))
            self.table_widget.setItem(self.table_widget.rowCount() - 1, 2, QTableWidgetItem(name))

        self.table_widget_lock.release()
        return self.table_widget.rowCount() - 1

    def row_double_clicked(self, row, column):
        if row >= 1:
            row_pid = self.table_widget.item(row, 0).text()
            node_info = self.running_nodes[row_pid]

            node_info_dialog = QDialog()
            node_info_dialog.setWindowTitle("Node Info")

            node_info_layout = QVBoxLayout(node_info_dialog)

            node_info_label = QLabel()

            node_info_text = ""
            node_info_text_fields = ["PID", "Package", "File", "Args"]
            for field in node_info_text_fields:
                if field != "Args":
                    node_info_text += f'<b>{field}:</b> {node_info[field]}<br>'
            node_info_text += '<b>Args: </b>'
            if node_info["Args"] == "":
                node_info_text += "None"
            node_info_label.setText(node_info_text)
            node_info_layout.addWidget(node_info_label)

            args_table = QTableWidget()

            args_table.setColumnCount(2)
            args_table.setHorizontalHeaderItem(0, QTableWidgetItem("Name"))
            args_table.setHorizontalHeaderItem(1, QTableWidgetItem("Value"))

            if node_info["Args"] != "":
                for arg in node_info["Args"].split(" "):
                    arg_name, arg_value = arg.split(":=")
                    args_table.insertRow(args_table.rowCount())
                    args_table.setItem(args_table.rowCount() - 1, 0, QTableWidgetItem(arg_name))
                    args_table.setItem(args_table.rowCount() - 1, 1, QTableWidgetItem(arg_value))

                node_info_layout.addWidget(args_table)

            node_info_dialog.exec()

    def closeEvent(self, event):
        self.launch_dialog.accept()

    def settings(self):
        settings = LaunchWidgetSettings(self, self.display_all_nodes)
        if settings.exec_():
            self.display_all_nodes = settings.get_values()


class LaunchWidgetSettings(QDialog):
    def __init__(self, parent, display_all_nodes):
        super().__init__(parent)

        self.setFixedSize(285, 70)

        self.display_all_nodes_checkbox = QCheckBox(self)
        self.display_all_nodes_checkbox.setChecked(display_all_nodes)
        self.display_all_nodes_label = QLabel("Display all nodes (?)", self)
        self.display_all_nodes_label.setToolTip("If checked, the plugin will display all nodes launched by "
                                                "remote_launch (not just nodes launched by the plugin) and allow "
                                                "the user to terminate any node that is displayed in the table. This "
                                                "means that if there is more than one instance of the Launch Plugin "
                                                "being used to launch nodes, then all instances will display all "
                                                "nodes that have been launched by all instances and they all can "
                                                "terminate each others' node as well.")

        buttonBox = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel, self)

        layout = QFormLayout(self)
        layout.addRow(self.display_all_nodes_label, self.display_all_nodes_checkbox)
        layout.addWidget(buttonBox)

        buttonBox.accepted.connect(self.accept)
        buttonBox.rejected.connect(self.reject)

    def get_values(self):
        return (self.display_all_nodes_checkbox.isChecked())
