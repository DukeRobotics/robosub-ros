#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import yaml
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from enum import Enum
import resource_retriever as rr
import sys


class Movement(Enum):
    TRANSLATION = 0
    ROTATION = 1


class JoystickParser(Node):
    NODE_NAME = 'joy_pub'
    JOYSTICK_RAW_TOPIC = 'joystick/raw'
    JOY_DEST_TOPIC = 'controls/desired_power'
    DEFAULT_JOYSTICK = 'F310'

    def __init__(self):
        super().__init__(self.NODE_NAME)
        self._pub = self.create_publisher(Twist, self.JOY_DEST_TOPIC, 50)
        self._current_joy_msg = Twist()
        self._movement_type = Movement.TRANSLATION

        joystick_type = self.declare_parameter(f'{self.NODE_NAME}/joystick_type', 
            self.DEFAULT_JOYSTICK).value
        self.get_logger().info(f'Starting {joystick_type} joystick')
        with open(rr.get_filename('package://joystick/config/joystick.yaml', use_protocol=False)) as f:
            data = yaml.safe_load(f)
        self._button_indices = data[joystick_type]

        self.create_subscription(Joy, self.JOYSTICK_RAW_TOPIC, self._parse_data, 10)

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

        # invert roll to match orientation given by right-hand rule
        self._current_joy_msg.angular.x = -self._leftLR
        self._current_joy_msg.angular.y = self._leftUD
        self._current_joy_msg.angular.z = self._rightLR

    def _publish_current_msg(self):
        self._pub.publish(self._current_joy_msg)


def main(args=None):
    try:
        rclpy.init(args=args)
        parser = JoystickParser()
        rclpy.spin(parser)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        sys.exit(1)
    finally:
        parser.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
