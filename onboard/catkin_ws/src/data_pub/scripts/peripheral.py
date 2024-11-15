#!/usr/bin/env python3

import rospy
import os
from std_msgs.msg import Float64
from std_srvs.srv import SetBool, SetBoolResponse
from serial_publisher import SerialPublisher

# Used for sensor fusion
from geometry_msgs.msg import PoseWithCovarianceStamped

CONFIG_FILE_PATH = f'package://offboard_comms/config/{os.getenv("ROBOT_NAME", "oogway")}.yaml'
CONFIG_NAME = 'peripheral'

BAUDRATE = 9600
NODE_NAME = 'peripheral_pub'
DEPTH_DEST_TOPIC = 'sensors/depth'
VOLTAGE_DEST_TOPIC = 'sensors/voltage'
HUMIDITY_DEST_TOPIC = 'sensors/humidity'
TEMPERATURE_DEST_TOPIC = 'sensors/temperature'
SERVO_SERVICE = 'servo_control'


class PeripheralPublisher(SerialPublisher):
    """
    Serial publisher to publish voltage and pressure data to ROS
    """

    MEDIAN_FILTER_SIZE = 3

    def __init__(self):
        super().__init__(NODE_NAME, BAUDRATE, CONFIG_FILE_PATH, CONFIG_NAME)

        # Pressure/Voltage
        self._pressure = None  # Pressure to publish
        self._previous_pressure = None  # Previous pressure readings for median filter

        self._pub_depth = rospy.Publisher(DEPTH_DEST_TOPIC, PoseWithCovarianceStamped, queue_size=50)
        self._pub_voltage = rospy.Publisher(VOLTAGE_DEST_TOPIC, Float64, queue_size=10)

        self._current_pressure_msg = PoseWithCovarianceStamped()
        self._current_voltage_msg = Float64()

        # Temperature/Servo
        self._temperature = None  # Temperature to publish
        self._humidity = None
        self._previous_temperature = None  # Previous temperature readings for median filter

        self._pub_temperature = rospy.Publisher(TEMPERATURE_DEST_TOPIC, Float64, queue_size=10)
        self._pub_humidity = rospy.Publisher(HUMIDITY_DEST_TOPIC, Float64, queue_size=10)

        self._servo_service = rospy.Service(SERVO_SERVICE, SetBool, self.servo_control)

        self._current_temperature_msg = Float64()
        self._current_humidity_msg = Float64()

        # Serial
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

    def servo_control(self, req):
        """
        Callback for servo control service

        @param req: the request to control the servo
        """
        rospy.logdebug('ServoControl received.')
        if req.data:
            self.writeline('L')
        else:
            self.writeline('R')
        return SetBoolResponse(True, f'Successfully set servo to {"left" if req.data else "right"}.')

    def process_line(self, line):
        """"
        Reads and publishes individual lines

        @param line: the line to read

        Assumes data comes in the following format:

        T:69.1
        H:30.1
        T:69.2
        H:27.8
        T:69.1
        H:27.8
        T:69.8
        ...
        """
        tag = line[0:2]  # T for temperature and H for humidity
        data = line[2:]
        if data == "":
            return
        if "T:" in tag:
            self._update_temperature(float(data))  # Filter out bad readings
            self._publish_current_temperature_msg()  # Publish temperature data
        if "H:" in tag:
            self._update_humidity(float(data))  # Filter out bad readings
            self._publish_current_humidity_msg()  # Publish humidity data
        rospy.sleep(1)

    def _update_temperature(self, new_reading):
        """
        Update temperature reading to publish and filter out bad readings

        @param new_reading: new temperature value to be printed
        """
        # Ignore readings that are too large
        if abs(new_reading) > 200:
            return

        # First reading
        elif self._temperature is None:
            self._temperature = new_reading
            self._previous_temperature = [new_reading] * self.MEDIAN_FILTER_SIZE

        # Median filter
        else:
            self._previous_temperature.append(new_reading)
            self._previous_temperature.pop(0)
            self._temperature = sorted(self._previous_temperature)[int(self.MEDIAN_FILTER_SIZE / 2)]

    def _update_humidity(self, new_reading):
        """
        Update humidity reading to publish and filter out bad readings

        @param new_reading: new humidity value to be printed
        """
        # Ignore readings that are too large
        if abs(new_reading) > 200:
            return

        # First reading
        elif self._humidity is None:
            self._humidity = new_reading
            self._previous_humidity = [new_reading] * self.MEDIAN_FILTER_SIZE

        # Median filter
        else:
            self._previous_humidity.append(new_reading)
            self._previous_humidity.pop(0)
            self._humidity = sorted(self._previous_humidity)[int(self.MEDIAN_FILTER_SIZE / 2)]

    def _publish_current_temperature_msg(self):
        """
        Publishes current temperature to ROS node
        """
        self._current_temperature_msg.data = self._temperature
        self._pub_temperature.publish(self._current_temperature_msg)

    def _publish_current_humidity_msg(self):
        """
        Publishes current humidity to ROS node
        """
        self._current_humidity_msg.data = self._humidity
        self._pub_humidity.publish(self._current_humidity_msg)

if __name__ == '__main__':
    try:
        PeripheralPublisher().run()
    except rospy.ROSInterruptException:
        pass
