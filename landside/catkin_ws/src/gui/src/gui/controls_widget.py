from __future__ import print_function

from std_srvs.srv import SetBool

import os
import time
import sys

import rospy
import rospkg

from geometry_msgs.msg import Pose, Twist

from gui_ssh_client import GUISSHClient

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget

#TODO make checkbox for sim mode

class ControlsWidget(QWidget):

    def __init__(self):

        super(ControlsWidget, self).__init__()

        ui_file = os.path.join(rospkg.RosPack().get_path('gui'), 'resource', 'ControlsInterface.ui')
        loadUi(ui_file, self)

        self.setObjectName('ControlsWidget')

        # Ties buttons to handler functions
        self.pose_enable.clicked.connect(self._handle_pose_button_clicked)
        self.twist_enable.clicked.connect(self._handle_twist_button_clicked)
        self.launch_enable.clicked.connect(self._handle_launch_file_button_clicked)
        self.controls_enable.clicked.connect(self._handle_controls_state_button_clicked)
        
        # All possible values for background color and text stored here
        self.background_colors = {
            "default" : "background-color: #ebfbff",
            "green"   : "background-color: #8aff92",
            "red"     : "background-color: #ff7878"
        }

        self.button_text = {
            "publish_disabled"  : "Start Publishing",
            "publish_enabled"   : "STOP Publishing",
            "launch_disabled"   : "Run Launch File",
            "launch_enabled"    : "STOP Launch File",
            "controls_disabled" : "Enable Controls",
            "controls_enabled"  : "Disable Controls",
        }

        self.pose_enable.setStyleSheet(self.background_colors["default"])
        self.twist_enable.setStyleSheet(self.background_colors["default"])
        self.launch_enable.setStyleSheet(self.background_colors["green"])
        self.controls_enable.setStyleSheet(self.background_colors["green"])


        # Holds state of whether actively running desired pose/twist
        self.pose_enable_state = False
        self.twist_enable_state = False
        self.launch_enable_state = False
        self.controls_enable_state = False

        self.x_pose.setSingleStep(0.1)
        self.y_pose.setSingleStep(0.1)
        self.z_pose.setSingleStep(0.1)
        self.roll_pose.setSingleStep(0.1)
        self.pitch_pose.setSingleStep(0.1)
        self.yaw_pose.setSingleStep(0.1)
        self.x_twist.setSingleStep(0.1)
        self.y_twist.setSingleStep(0.1)
        self.z_twist.setSingleStep(0.1)
        self.roll_twist.setSingleStep(0.1)
        self.pitch_twist.setSingleStep(0.1)
        self.yaw_twist.setSingleStep(0.1)


    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    # our methods
    def disable_pose(self):
        self.publish_pose_client.stop()
        self.pose_enable_state = False
        
        # Set the button color and text
        self.pose_enable.setStyleSheet(self.background_colors["default"])
        self.pose_enable.setText(self.button_text["publish_disabled"])
    
    def disable_twist(self):
        self.publish_twist_client.stop()
        self.twist_enable_state = False

        # Set the button color and text
        self._widget.twist_enable.setStyleSheet(self.background_colors["default"])
        self._widget.twist_enable.setText(self.button_text["publish_disabled"])

    def enable_pose(self):
        self.pose_enable_state = True

        self.publish_pose_client.set_linear_x(self._widget.x_pose.value())
        self.publish_pose_client.set_linear_y(self._widget.y_pose.value())
        self.publish_pose_client.set_linear_z(self._widget.z_pose.value())
        self.publish_pose_client.set_angular_x(self._widget.roll_pose.value())
        self.publish_pose_client.set_angular_y(self._widget.pitch_pose.value())
        self.publish_pose_client.set_angular_z(self._widget.yaw_pose.value())
        
        self.publish_pose_client.execute_pose()

        # Set the button color and text
        self._widget.pose_enable.setStyleSheet(self.background_colors["red"])
        self._widget.pose_enable.setText(self.button_text["publish_enabled"])

    def enable_twist(self):
        self.twist_enable_state = True

        self.publish_twist_client.set_linear_x(self._widget.x_twist.value())
        self.publish_twist_client.set_linear_y(self._widget.y_twist.value())
        self.publish_twist_client.set_linear_z(self._widget.z_twist.value())
        self.publish_twist_client.set_angular_x(self._widget.roll_twist.value())
        self.publish_twist_client.set_angular_y(self._widget.pitch_twist.value())
        self.publish_twist_client.set_angular_z(self._widget.yaw_twist.value())
        
        self.publish_twist_client.execute_twist()

        # Set the button color and text
        self._widget.twist_enable.setStyleSheet(self.background_colors["red"])
        self._widget.twist_enable.setText(self.button_text["publish_enabled"])

    def _handle_pose_button_clicked(self):
        if self.pose_enable_state:
            # pose was enabled, disable pose
            self.disable_pose()

        else:
            # pose was disabled, make sure twist is off and enable pose
            if self.twist_enable_state:
                self.disable_twist()
            
            self.enable_pose()
    
    def _handle_twist_button_clicked(self):
        if self.twist_enable_state:
            # twist was enabled, disable twist
            self.disable_twist()

        else:
            # twist was disabled, make sure pose is off and enable twist
            if self.pose_enable_state:
                self.disable_pose()
            
            self.enable_twist()
        
        
    def _handle_launch_file_button_clicked(self): 
        self.launch_enable_state = not self.launch_enable_state
        if self.launch_enable_state:
            self.launch_file_client.execute_launch_file()

            # Set colors and text
            self._widget.launch_enable.setStyleSheet(self.background_colors["red"])
            self._widget.launch_enable.setText(self.button_text["launch_enabled"])
        else:
            self.launch_file_client.stop()

            # Set colors and text
            self._widget.launch_enable.setStyleSheet(self.background_colors["green"])
            self._widget.launch_enable.setText(self.button_text["launch_disabled"])

    # TODO time out so gui doesnt freeze when service nonexistant
    def _handle_controls_state_button_clicked(self):
        self.controls_enable_state = not self.controls_enable_state

        # Set colors and text
        if self.controls_enable_state:
            self._widget.controls_enable.setStyleSheet(self.background_colors["red"])
            self._widget.controls_enable.setText(self.button_text["controls_enabled"])
        else:
            self._widget.controls_enable.setStyleSheet(self.background_colors["green"])
            self._widget.controls_enable.setText(self.button_text["controls_disabled"])

        rospy.wait_for_service('enable_controls')
        try:
            enable_controls = rospy.ServiceProxy('enable_controls', SetBool)
            res = enable_controls(self.controls_enable_state) # TODO gui will need to say something about failure for res
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e) 
    
    # TODO def _handle_sim_clicked(self):
        #self.sim_mode = self.sim_checkbox.isChecked()