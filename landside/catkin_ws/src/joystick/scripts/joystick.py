#!/usr/bin/env python

import rospy
import yaml
import sys
import os
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from enum import Enum


class Movement(Enum):
    TRANSLATION = 0
    ROTATION = 1


class JoystickParser:
    NODE_NAME = 'joy_pub'
    JOYSTICK_RAW_TOPIC = 'joystick/raw'
    JOY_DEST_TOPIC = 'controls/desired_twist_power'

    def __init__(self):
        self._pub = rospy.Publisher(self.JOY_DEST_TOPIC, Twist, queue_size=50)
        self._current_joy_msg = Twist()
        self._movement_type = Movement.TRANSLATION

        joystick_type = rospy.get_param("~/joy_pub/joystick_type")
        with open(os.path.join(sys.path[0], '../config/joystick.yaml')) as f:
            data = yaml.load(f)
        self._button_indices = data[joystick_type]

        rospy.init_node(self.NODE_NAME)
        rospy.Subscriber(self.JOYSTICK_RAW_TOPIC, Joy, self._parse_data)
        rospy.spin()

    def _parse_data(self, raw_joystick_data):
        self._read_joystick_data(raw_joystick_data)
        if self._movement_type == Movement.TRANSLATION:
            self._parse_linear()
        else:
            self._parse_angular()
        self._publish_current_msg()

    def _read_joystick_data(self, raw_joystick_data):
        self._leftLR = raw_joystick_data.axes[0]
        self._leftUD = raw_joystick_data.axes[1]
        self._rightLR = raw_joystick_data.axes[2]
        self._rightUD = raw_joystick_data.axes[3]

        if raw_joystick_data.buttons[self._get_linear_button()] == 1:
            self._movement_type = Movement.TRANSLATION

        if raw_joystick_data.buttons[self._get_rotation_button()] == 1:
            self._movement_type = Movement.ROTATION

    def _get_linear_button(self):
        return self._button_indices["linearButton"]

    def _get_rotation_button(self):
        return self._button_indices["rotButton"]

    def _parse_linear(self):
        self._current_joy_msg.linear.x = self._leftUD
        self._current_joy_msg.linear.y = self._leftLR
        self._current_joy_msg.linear.z = self._rightUD

        self._current_joy_msg.angular.x = 0
        self._current_joy_msg.angular.y = 0
        self._current_joy_msg.angular.z = self._rightLR

    def _parse_angular(self):
        self._current_joy_msg.linear.x = 0
        self._current_joy_msg.linear.y = 0
        self._current_joy_msg.linear.z = self._rightUD

        self._current_joy_msg.angular.x = -self._leftLR  # invert roll to match orientation given by right-hand rule
        self._current_joy_msg.angular.y = self._leftUD
        self._current_joy_msg.angular.z = self._rightLR

    def _publish_current_msg(self):
        self._pub.publish(self._current_joy_msg)


if __name__ == '__main__':
    try:
        parser = JoystickParser()
    except rospy.ROSInterruptException:
        pass
