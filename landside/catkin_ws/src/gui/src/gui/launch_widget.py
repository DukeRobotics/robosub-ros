import os
import glob

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QDialogButtonBox
from python_qt_binding.QtCore import pyqtSignal

import rospy
import resource_retriever as rr

from custom_msgs.srv import StartLaunch


class LaunchWidget(QWidget):

    ROOT_PATH = '/home/dev/robosub-ros/onboard/catkin_ws/src'

    node_launched = pyqtSignal(int, str, str, str, name='nodeLaunched')

    def __init__(self):
        super(LaunchWidget, self).__init__()

        ui_file = rr.get_filename('package://gui/resource/LaunchWidget.ui', use_protocol=False)
        loadUi(ui_file, self)

        self.setObjectName('LaunchWidget')

        package_list = glob.glob(os.path.join(self.ROOT_PATH, '*/'))
        package_names = [os.path.split(f)[1] for f in package_list]
        self.package_name_box.addItems(package_names)
        self.package_name_box.setEnabled(True)
        self.package_name_box.activated.connect(self.package_name_selected)

        self.node_name_box.setEnabled(False)
        self.node_name_box.activated.connect(self.node_name_selected)

        self.accept_button.setEnabled(False)
        self.accept_button.clicked.connect(self.accept)

        self.cancel_button.setEnabled(True)
        self.cancel_button.clicked.connect(self.close)

    def get_launchables(self, package_name):
        package_dir = os.path.join(self.ROOT_PATH, package_name)
        launch_files = glob.glob(os.path.join(package_dir, 'launch', '*.launch'))
        script_files = glob.glob(os.path.join(package_dir, 'scripts', '*.py'))
        executables = [f for f in script_files if os.access(f, os.X_OK)]
        return launch_files + executables

    def package_name_selected(self, item_index):
        self.node_name_box.clear()

        if item_index == -1:
            self.node_name_box.setEnabled(False)
            return

        selected_package = self.package_name_box.itemText(item_index)
        launchables = self.get_launchables(selected_package)
        launchable_names = [os.path.split(f)[1] for f in launchables]
        self.node_name_box.addItems(launchable_names)
        self.node_name_box.setEnabled(True)

    def node_name_selected(self, item_index):
        if item_index == -1:
            self.accept_button.setEnabled(False)
        else:
            self.accept_button.setEnabled(True)

    def accept(self):
        try:
            rospy.wait_for_service('start_node', 5)
        except rospy.RosException:
            rospy.logerr('start_node service not active')
            return
        package = self.package_name_box.currentText()
        node = self.node_name_box.currentText()
        args = self.args_input.text()
        start_launch = rospy.ServiceProxy('start_node', StartLaunch)
        try:
            resp = start_launch(package, node, args, node.endswith('.launch'))
            self.node_launched.emit(resp.pid, package, node, args)
        except rospy.ServiceException as exc:
            rospy.logerr(f'Service did not process request: {str(exc)}')
        
        self.close()
