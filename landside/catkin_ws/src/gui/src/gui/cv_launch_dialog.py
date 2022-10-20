import os
import glob

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QDialog
from python_qt_binding.QtCore import pyqtSignal

import rospy
import resource_retriever as rr

from custom_msgs.srv import StartLaunch


class CVLaunchDialog(QDialog):

    ROOT_PATH = '/root/dev/robosub-ros/onboard/catkin_ws/src'

    node_launched = pyqtSignal(int, str, str, str, name='nodeLaunched')

    def __init__(self):
        super(CVLaunchDialog, self).__init__()

        ui_file = rr.get_filename('package://gui/resource/CVLaunchDialog.ui', use_protocol=False)
        loadUi(ui_file, self)

        self.reset()

        self.package_name_box.activated.connect(self.package_name_selected)
        self.node_name_box.activated.connect(self.node_name_selected)
        self.accept_button.clicked.connect(self.click_ok)
        self.cancel_button.clicked.connect(self.reject)

    def reset(self):
        package_list = glob.glob(os.path.join(self.ROOT_PATH, '*/'))
        package_names = ['cv', '']
        for f in package_list:
            package = str(os.path.split(f[:-1])[1])
            if package != 'cv':
                package_names.append(package)
        # package_names = ['cv'] + [str(os.path.split(f[:-1])[1]) for f in package_list]
        self.package_name_box.clear()
        self.package_name_box.addItems(package_names)
        self.package_name_box.setEnabled(True)
        self.node_name_box.setEnabled(False)
        self.accept_button.setEnabled(False)
        self.cancel_button.setEnabled(True)

        self.set_node_name_box('cv')

    def get_launchables(self, package_name):
        package_dir = os.path.join(self.ROOT_PATH, package_name)
        launch_files = glob.glob(os.path.join(package_dir, 'launch', '*.launch'))
        script_files = glob.glob(os.path.join(package_dir, 'scripts', '*.py'))
        executables = [f for f in script_files if os.access(f, os.X_OK)]
        return [''] + launch_files + executables

    def package_name_selected(self, item_index):
        if item_index == 1:
            self.node_name_box.setEnabled(False)
            return

        selected_package = self.package_name_box.itemText(item_index)
        self.set_node_name_box(selected_package)

    def set_node_name_box(self, selected_package):
        self.node_name_box.clear() 
        launchables = self.get_launchables(selected_package)
        launchable_names = [os.path.split(f)[1] for f in launchables]
        self.node_name_box.addItems(launchable_names)
        self.node_name_box.setEnabled(True)

    def node_name_selected(self, item_index):
        if item_index == 0:
            self.accept_button.setEnabled(False)
        else:
            self.accept_button.setEnabled(True)

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
            self.reject()
        else:
            self.accept()
