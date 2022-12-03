import os
import glob

from python_qt_binding import loadUi, QtWidgets
from python_qt_binding.QtWidgets import QDialog, QMessageBox
from python_qt_binding.QtCore import pyqtSignal, pyqtProperty

import rospy
import resource_retriever as rr

from custom_msgs.srv import StartLaunch

import xml.etree.ElementTree as ET


class LaunchDialog(QDialog):

    ROOT_PATH = '/root/dev/robosub-ros/onboard/catkin_ws/src'

    node_launched = pyqtSignal(int, str, str, str, name='nodeLaunched')

    def __init__(self, p):
        super(LaunchDialog, self).__init__()

        ui_file = rr.get_filename('package://gui/resource/LaunchDialog.ui', use_protocol=False)
        loadUi(ui_file, self)

        self.parent = p
        self.default_package = ''

        self.reset()

        self.package_name_box.activated.connect(self.package_name_selected)
        self.node_name_box.activated.connect(self.node_name_selected)
        self.accept_button.clicked.connect(self.click_ok)

        self.arg_form_rows = []

    @pyqtProperty(str)
    def default_pkg(self):
        return self.default_package

    @default_pkg.setter
    def default_pkg(self, value):
        self.default_package = value

        # Making self.default_package selected by default
        self.package_name_box.setCurrentText(self.default_package)
        self.setup_node_name_box(self.default_package)
        self.node_name_box.setEnabled(bool(self.default_package))

    def get_package_names(self):
        package_list = glob.glob(os.path.join(self.ROOT_PATH, '*/'))
        return [''] + [str(os.path.split(f[:-1])[1]) for f in package_list]

    def reset(self):
        self.package_name_box.clear()
        self.package_name_box.addItems(self.get_package_names())
        self.package_name_box.setEnabled(True)
        self.node_name_box.clear()
        self.node_name_box.setEnabled(False)
        self.args_input.clear()
        self.accept_button.setEnabled(False)

    def get_launchables(self, package_name):
        if package_name:
            package_dir = os.path.join(self.ROOT_PATH, package_name)
            launch_files = glob.glob(os.path.join(package_dir, 'launch', '*.launch'))
            script_files = glob.glob(os.path.join(package_dir, 'scripts', '*.py'))
            executables = [f for f in script_files if os.access(f, os.X_OK)]
            return [''] + launch_files + executables
        else:
            return []

    def package_name_selected(self, item_index):
        if item_index == 0:
            self.node_name_box.setEnabled(False)
            return

        selected_package = self.package_name_box.itemText(item_index)
        self.setup_node_name_box(selected_package)
        self.node_name_box.setEnabled(True)

    def setup_node_name_box(self, selected_package):
        self.node_name_box.clear()
        launchables = self.get_launchables(selected_package)
        launchable_names = [os.path.split(f)[1] for f in launchables]
        self.node_name_box.addItems(launchable_names)

    def node_name_selected(self, item_index):
        for row in self.arg_form_rows:
            self.form_layout.removeRow(row['label'])

        if item_index == 0:
            self.accept_button.setEnabled(False)
            return
        else:
            self.accept_button.setEnabled(True)

        self.arg_form_rows = []

        selected_node = self.node_name_box.currentText()
        selected_node_file_type = selected_node.split(".")[1]

        if selected_node_file_type == "launch":
            package_dir = os.path.join(self.ROOT_PATH, self.package_name_box.currentText())
            launch_file_path = os.path.join(package_dir, 'launch/' + self.node_name_box.currentText())

            tree = ET.parse(launch_file_path)

            def traverse_tree(root):
                for child in root:
                    if child.tag == 'arg' and child.get('value') is None:
                        self.arg_form_rows.append(child.attrib)
                    traverse_tree(child)

            root = tree.getroot()
            traverse_tree(root)

            for row in range(len(self.arg_form_rows)):
                arg = self.arg_form_rows[row]
                if arg.get('default') is None:
                    default_value = ''
                else:
                    default_value = arg['default']

                label = QtWidgets.QLabel(arg['name'])
                input = QtWidgets.QLineEdit()
                input.setText(default_value)

                # row inserted at position row+2, after the Package and Node Name rows
                self.form_layout.insertRow(row + 2, label, input)

                arg['label'] = label
                arg['lineEdit'] = input

    def click_ok(self):
        package = self.package_name_box.currentText()
        node = self.node_name_box.currentText()
        args = self.args_input.text().split(' ') if self.args_input.text() != "" else []

        for row in self.arg_form_rows:
            if row['lineEdit'].text() == "" and row.get('default') is None:
                self.missing_argument_dialog(row['name'])
                return
            arg = row['name'] + ":=" + row['lineEdit'].text()
            args.append(arg)

        start_launch = rospy.ServiceProxy('start_node', StartLaunch)
        try:
            resp = start_launch(package, node, args, node.endswith('.launch'))
            self.node_launched.emit(resp.pid, package, node, " ".join(args))
        except rospy.ServiceException as exc:
            rospy.logerr(f'Service did not process request: {str(exc)}')

    def missing_argument_dialog(self, missing_arg):
        msg = QMessageBox()

        msg.setIcon(QMessageBox.Warning)

        msg.setWindowTitle("Error")
        msg.setText("Missing argument")
        msg.setInformativeText("You must specify a value for " + missing_arg)

        msg.setStandardButtons(QMessageBox.Close)

        msg.exec_()

    def reject(self):
        pass
