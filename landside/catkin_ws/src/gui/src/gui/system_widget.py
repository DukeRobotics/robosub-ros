import os
import glob

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QDialogButtonBox, QDialog, QAbstractItemView, QTableWidgetItem
from python_qt_binding.QtCore import QEvent
import python_qt_binding.QtCore as QtCore

import rospy
import resource_retriever as rr

from gui.launch_widget import LaunchWidget
from custom_msgs.msg import SystemUsage
from custom_msgs.srv import StartLaunch, StopLaunch


class SystemWidget(QWidget):

    LAUNCHFILES = {'motion' : 'motion.launch',
                   'state' : 'state.launch', 
                   'tasks' : 'tasks.launch'}

    def __init__(self):
        super(SystemWidget, self).__init__()

        ui_file = rr.get_filename('package://gui/resource/SystemWidget.ui', use_protocol=False)
        loadUi(ui_file, self)

        self.simulation = False
        self.simulation_check_box.setChecked(False)
        self.simulation_check_box.stateChanged.connect(self.simulation_changed)

        self.cpu_value.setDigitCount(4)
        self.cpu_value.display(0)
        self.ram_value.setDigitCount(4)
        self.ram_value.display(0)
        self.system_sub = rospy.Subscriber('/system/usage', SystemUsage, self.update_cpu_ram)

        self.execute_buttons = {"motion" : self.motion_button,
                                "state": self.state_button,
                                "tasks": self.tasks_button}
        self.execute_rows = {"motion" : 0,
                             "state" : 0,
                             "tasks": 0}

        self.motion_button.setText(f'Start {self.LAUNCHFILES["motion"]}')
        self.motion_button.clicked.connect(lambda: self.execute_button_clicked('motion'))

        self.state_button.setText(f'Start {self.LAUNCHFILES["state"]}')
        self.state_button.clicked.connect(lambda: self.execute_button_clicked('state'))

        self.tasks_button.setText(f'Start {self.LAUNCHFILES["tasks"]}')
        self.tasks_button.clicked.connect(lambda: self.execute_button_clicked('tasks'))

        self.table_widget.setSelectionBehavior(QAbstractItemView.SelectRows)

        self.launch_dialog = LaunchWidget()
        self.launch_dialog.setObjectName('LaunchWidget')
        self.launch_dialog.reset()
        self.launch_dialog.node_launched.connect(self.append_to_table)

        self.launch_dialog_button.clicked.connect(self.launch_node_dialog)

        rospy.wait_for_service('start_node')
        rospy.wait_for_service('stop_node')

        rospy.loginfo('System Widget successfully initialized')
    
    def keyPressEvent(self, event):
        if event.key() == QtCore.Qt.Key_Delete:
            items = self.table_widget.selectedItems()
            if items:
                self.delete_launch(items[0].row())
        super(SystemWidget, self).keyPressEvent(event)

    def simulation_changed(self, val):
        self.simulation = self.simulation_check_box.isChecked()

    def update_cpu_ram(self, system_usage):
        self.cpu_value.display(system_usage.cpu_percent)
        self.ram_value.display(system_usage.ram.used)
    
    def launch_node_dialog(self):
        self.launch_dialog.reset()
        self.launch_dialog.open()
    
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
        self.table_widget.setItem(self.table_widget.rowCount()-1, 0, QTableWidgetItem(str(pid)))
        self.table_widget.setItem(self.table_widget.rowCount()-1, 1, QTableWidgetItem(package))
        self.table_widget.setItem(self.table_widget.rowCount()-1, 2, QTableWidgetItem(name))
        self.table_widget.setItem(self.table_widget.rowCount()-1, 3, QTableWidgetItem(args))
        return self.table_widget.rowCount() - 1

    def execute_button_clicked(self, button):
        start_launch = self.execute_buttons[button].text() == f'Start {self.LAUNCHFILES[button]}'
        for key in self.execute_buttons:
            if key != button:
                self.execute_buttons[key].setEnabled(not start_launch)
        new_button_text = 'Stop ' if start_launch else 'Start '
        self.execute_buttons[button].setText(new_button_text + self.LAUNCHFILES[button])
        if start_launch:
            self.execute_rows[button] = self.launch_file(self.LAUNCHFILES[button])
        else:
            self.delete_launch(self.execute_rows[button])
            self.execute_rows[button] = 0
