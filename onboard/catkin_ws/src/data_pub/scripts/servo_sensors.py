#!/usr/bin/env python3

import rospy
import os
from std_msgs.msg import Float64
from std_srvs.srv import SetBool, SetBoolResponse
from serial_publisher import SerialPublisher

CONFIG_FILE_PATH = f'package://offboard_comms/config/{os.getenv("ROBOT_NAME", "oogway")}.yaml'
CONFIG_NAME = 'servo_sensors'

BAUDRATE = 9600
NODE_NAME = 'servo_sensors_pub'
HUMIDITY_DEST_TOPIC = 'sensors/humidity'
TEMPERATURE_DEST_TOPIC = 'sensors/temperature'
SERVO_SERVICE = 'servo_control'


class TemperatureHumidityPublisher(SerialPublisher):
    """
    Serial publisher to publish temperature and humidity data to ROS
    """

    MEDIAN_FILTER_SIZE = 3

    def __init__(self):
        super().__init__(NODE_NAME, BAUDRATE, CONFIG_FILE_PATH, CONFIG_NAME)

        self._temperature = None  # Temperature to publish
        self._humidity = None
        self._previous_temperature = None  # Previous temperature readings for median filter

        self._pub_temperature = rospy.Publisher(TEMPERATURE_DEST_TOPIC, Float64, queue_size=10)
        self._pub_humidity = rospy.Publisher(HUMIDITY_DEST_TOPIC, Float64, queue_size=10)

        self._servo_service = rospy.Service(SERVO_SERVICE, SetBool, self.servo_control)

        self._current_temperature_msg = Float64()
        self._current_humidity_msg = Float64()

        self._serial_port = None
        self._serial = None

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
        TemperatureHumidityPublisher().run()
    except rospy.ROSInterruptException:
        pass
