from ast import arg
from email.policy import default
from multiprocessing.spawn import is_forking
import os
import glob

from python_qt_binding import loadUi, QtWidgets
from python_qt_binding.QtWidgets import QDialog
from python_qt_binding.QtCore import pyqtSignal, pyqtProperty

import rospy
import resource_retriever as rr

from custom_msgs.srv import StartLaunch

import xml.etree.ElementTree as ET

class CVLaunchDialog(QDialog):

    ROOT_PATH = '/root/dev/robosub-ros/onboard/catkin_ws/src'

    node_launched = pyqtSignal(int, str, str, str, name='nodeLaunched')

    argFormRows = []

    def __init__(self, p):
        super(CVLaunchDialog, self).__init__()

        ui_file = rr.get_filename('package://gui/resource/CVLaunchDialog.ui', use_protocol=False)
        loadUi(ui_file, self)

        self.parent = p
        self.default_package = ''

        self.reset()

        self.package_name_box.activated.connect(self.package_name_selected)
        self.node_name_box.activated.connect(self.node_name_selected)
        self.accept_button.clicked.connect(self.click_ok)
        # self.cancel_button.clicked.connect(self.reject)

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
        #self.cancel_button.setEnabled(True)

    def get_launchables(self, package_name):
        package_dir = os.path.join(self.ROOT_PATH, package_name)
        launch_files = glob.glob(os.path.join(package_dir, 'launch', '*.launch'))
        script_files = glob.glob(os.path.join(package_dir, 'scripts', '*.py'))
        executables = [f for f in script_files if os.access(f, os.X_OK)]
        return [''] + launch_files + executables

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
        if item_index == 0:
            self.accept_button.setEnabled(False)
        else:
            self.accept_button.setEnabled(True)

        # TODO: Step 3 - Remove all widgets in the self.argFormRows list from the UI.
        for row in self.argFormRows:
            self.form_layout.removeRow(row[0])
        self.argFormRows = []

        selected_node =  self.node_name_box.currentText()
        selected_node_file_type = selected_node.split(".")[1]

        if selected_node_file_type == "launch":
            package_dir = os.path.join(self.ROOT_PATH, self.package_name_box.currentText())
            launch_file_path = os.path.join(package_dir, 'launch/' + self.node_name_box.currentText())

            # This is a list of arguments accepted by selected_node
            # A list of dictionaries of the form: [{name: 'arg1', default: ''}, {name: 'arg2', default: 'val2'}, {name: 'arg3', default: 'val3'} ... ]
            # If an argument has the 'value' property specified, it should NOT be included in this list
            args = []

            tree = ET.parse(launch_file_path)

            def traverse_tree(root):
                for child in root:
                    # TODO: Step 1 - Populate the args list above
                    # If child is an arg tag AND does not have the 'value' property specified, add it to the args list
                    # child.tag - the tag of child
                    # child.attrib - a dictionary with child's attribute names as keys and attribute values as values
                    if child.tag == 'arg' and child.get('value') is None:
                        args.append(child.attrib)
                    
                    traverse_tree(child)
            
            root = tree.getroot()
            traverse_tree(root)

            # TODO: Step 2 - For each arg in args, add a row to the form with that input value
            # The 'name' of the arg should be the Label
            # The 'defaultValue' of the arg (if any) should be the default value of the LineEdit
            # Add the QLabel and QLineEdit widgets to the self.argFormRows list

            # Example code to add row to form with label "Sample" and default value "sample_default_value"
            # sample_label = QtWidgets.QLabel("Sample")
            # sample_input = QtWidgets.QLineEdit()
            # sample_input.setText("sample_default_value")
            # self.form_layout.addRow(sample_label, sample_input)
            for i in range(len(args)):
                if args[i].get('default') is None:
                    default_value = ''
                else:
                    default_value = args[i]['default']
                
                label = QtWidgets.QLabel(args[i]['name'])
                input = QtWidgets.QLineEdit()
                input.setText(default_value)
                
                # row inserted at position i+2, after the Package and Node Name rows
                self.form_layout.insertRow(i+2, label, input)
                
                self.argFormRows.append([label, input])

    def click_ok(self):
        package = self.package_name_box.currentText()
        node = self.node_name_box.currentText()
        args = self.args_input.text().split(' ')
        start_launch = rospy.ServiceProxy('start_node', StartLaunch)
        try:
            resp = start_launch(package, node, args, node.endswith('.launch'))
            self.node_launched.emit(resp.pid, package, node, self.args_input.text())
        except rospy.ServiceException as exc:
            rospy.logerr(f'Service did not process request: {str(exc)}')