#!/usr/bin/env python3

import rospy
import time
import serial
import serial.tools.list_ports as list_ports
import yaml
import resource_retriever as rr
import traceback
from std_msgs.msg import Float64

# Used for sensor fusion
from geometry_msgs.msg import PoseWithCovarianceStamped


class PressureRawPublisher:

    DEPTH_DEST_TOPIC = 'sensors/depth'
    VOLTAGE_DEST_TOPIC = 'sensors/voltage'
    FTDI_FILE_PATH = 'package://offboard_comms/config/arduino.yaml'

    BAUDRATE = 9600
    NODE_NAME = 'pressure_pub'

    MEDIAN_FILTER_SIZE = 3

    def __init__(self):
        with open(rr.get_filename(self.FTDI_FILE_PATH, use_protocol=False)) as f:
            self._arduino_config = yaml.safe_load(f)

        self._pressure = None  # Pressure to publish
        self._previous_pressure = None  # Previous pressure readings for median filter

        self._pub_depth = rospy.Publisher(self.DEPTH_DEST_TOPIC, PoseWithCovarianceStamped, queue_size=50)
        self._pub_voltage = rospy.Publisher(self.VOLTAGE_DEST_TOPIC, Float64, queue_size=10)

        self._current_pressure_msg = PoseWithCovarianceStamped()
        self._current_voltage_msg = Float64()

        self._serial_port = None
        self._serial = None

    # Read FTDI strings of all ports in list_ports.grep
    def connect(self):
        while self._serial_port is None and not rospy.is_shutdown():
            try:
                pressure_ftdi_string = self._arduino_config['pressure']['ftdi']
                self._serial_port = next(list_ports.grep(pressure_ftdi_string)).device
                self._serial = serial.Serial(self._serial_port, self.BAUDRATE,
                                             timeout=1, write_timeout=None,
                                             bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                                             stopbits=serial.STOPBITS_ONE)
            except StopIteration:
                rospy.logerr("Pressure sensor not found, trying again in 0.1 seconds.")
                rospy.sleep(0.1)

    # Read line from serial port without blocking
    def readline_nonblocking(self, tout=1):
        start = time.time()
        buff = b''
        while ((time.time() - start) < tout) and (b'\r\n' not in buff):
            try:
                buff += self._serial.read(1)
            except serial.SerialException:
                pass

        return buff.decode('utf-8', errors='ignore')

    def run(self):
        rospy.init_node(self.NODE_NAME)
        self.connect()
        while not rospy.is_shutdown():
            try:
                # Direct read from device
                line = self.readline_nonblocking().strip()

                if not line or line == '':
                    rospy.logerr("Timeout in pressure serial read, trying again in 1 second.")
                    rospy.sleep(0.1)
                    continue  # Skip and retry

                tag = line[0:2]  # P for pressure and V for voltage
                data = line[2:]
                if "P:" in tag:
                    self._update_pressure(float(data))  # Filter out bad readings
                    self._parse_pressure()  # Parse pressure data
                    self._publish_current_pressure_msg()  # Publish pressure data
                if "V:" in tag:
                    self._current_voltage_msg = float(data)
                    self._publish_current_voltage_msg()

            except Exception:
                rospy.logerr("Error in reading pressure information. Reconnecting.")
                rospy.logerr(traceback.format_exc())
                self._serial.close()
                self._serial = None
                self._serial_port = None
                self.connect()

    # Update pressure reading to publish and filter out bad readings
    def _update_pressure(self, new_reading):
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
        # Pressure data recieved is positive so must flip sign

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
        if abs(self._current_pressure_msg.pose.pose.position.z) > 7:
            self._current_pressure_msg = PoseWithCovarianceStamped()
            return

        self._current_pressure_msg.header.stamp = rospy.Time.now()
        self._current_pressure_msg.header.frame_id = "odom"  # World frame

        self._pub_depth.publish(self._current_pressure_msg)
        self._current_pressure_msg = PoseWithCovarianceStamped()

    def _publish_current_voltage_msg(self):
        self._pub_voltage.publish(self._current_voltage_msg)


if __name__ == '__main__':
    try:
        PressureRawPublisher().run()
    except rospy.ROSInterruptException:
        pass
