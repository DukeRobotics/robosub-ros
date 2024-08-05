#!/usr/bin/env python3

import rospy
import os
from std_msgs.msg import Float64
from serial_publisher import SerialPublisher

# Used for sensor fusion
from geometry_msgs.msg import PoseWithCovarianceStamped

CONFIG_FILE_PATH = f'package://offboard_comms/config/{os.getenv("ROBOT_NAME", "oogway")}.yaml'
CONFIG_NAME = 'pressure'

BAUDRATE = 9600
NODE_NAME = 'pressure_voltage_pub'
DEPTH_DEST_TOPIC = 'sensors/depth'
VOLTAGE_DEST_TOPIC = 'sensors/voltage'


class PressureVoltagePublisher(SerialPublisher):
    """
    Serial publisher to publish voltage and pressure data to ROS
    """

    MEDIAN_FILTER_SIZE = 3

    def __init__(self):
        super().__init__(NODE_NAME, BAUDRATE, CONFIG_FILE_PATH, CONFIG_NAME)

        self._pressure = None  # Pressure to publish
        self._previous_pressure = None  # Previous pressure readings for median filter

        self._pub_depth = rospy.Publisher(DEPTH_DEST_TOPIC, PoseWithCovarianceStamped, queue_size=50)
        self._pub_voltage = rospy.Publisher(VOLTAGE_DEST_TOPIC, Float64, queue_size=10)

        self._current_pressure_msg = PoseWithCovarianceStamped()
        self._current_voltage_msg = Float64()

        self._serial_port = None
        self._serial = None

    def process_line(self, line):
        """"
        Reads and publishes individual lines

        @param line: the line to read

        Assumes data comes in the following format:

        P:0.22
        P:0.23
        P:0.22
        P:0.22
        V:15.85
        P:0.24
        ...
        """
        tag = line[0:2]  # P for pressure and V for voltage
        data = line[2:]
        if "P:" in tag:
            self._update_pressure(float(data))  # Filter out bad readings
            self._parse_pressure()  # Parse pressure data
            self._publish_current_pressure_msg()  # Publish pressure data
        if "V:" in tag:
            self._current_voltage_msg = float(data)
            self._publish_current_voltage_msg()

    def _update_pressure(self, new_reading):
        """
        Update pressure reading to publish and filter out bad readings

        @param new_reading: new pressure value to be printed
        """
        # Ignore readings that are too large
        if abs(new_reading) > 7:
            return

        # First reading
        elif self._pressure is None:
            self._pressure = new_reading
            self._previous_pressure = [new_reading] * self.MEDIAN_FILTER_SIZE

        # Median filter
        else:
            self._previous_pressure.append(new_reading)
            self._previous_pressure.pop(0)
            self._pressure = sorted(self._previous_pressure)[int(self.MEDIAN_FILTER_SIZE / 2)]

    def _parse_pressure(self):
        """
        Parses the pressure into an odom message
        """
        self._current_pressure_msg.pose.pose.position.x = 0.0
        self._current_pressure_msg.pose.pose.position.y = 0.0
        self._current_pressure_msg.pose.pose.position.z = -1 * float(self._pressure)

        self._current_pressure_msg.pose.pose.orientation.x = 0.0
        self._current_pressure_msg.pose.pose.orientation.y = 0.0
        self._current_pressure_msg.pose.pose.orientation.z = 0.0
        self._current_pressure_msg.pose.pose.orientation.w = 1.0

        # Only the z,z covariance
        self._current_pressure_msg.pose.covariance[14] = 0.01

    def _publish_current_pressure_msg(self):
        """
        Publishes current pressure to ROS node
        """
        if abs(self._current_pressure_msg.pose.pose.position.z) > 7:
            self._current_pressure_msg = PoseWithCovarianceStamped()
            return

        self._current_pressure_msg.header.stamp = rospy.Time.now()
        self._current_pressure_msg.header.frame_id = "odom"  # World frame

        self._pub_depth.publish(self._current_pressure_msg)
        self._current_pressure_msg = PoseWithCovarianceStamped()

    def _publish_current_voltage_msg(self):
        """
        Publishes current voltage to ROS node
        """
        self._pub_voltage.publish(self._current_voltage_msg)


if __name__ == '__main__':
    try:
        PressureVoltagePublisher().run()
    except rospy.ROSInterruptException:
        pass
