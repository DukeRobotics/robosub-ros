#!/usr/bin/env python3

import rospy
import time
import serial
import serial.tools.list_ports as list_ports
import yaml
import resource_retriever as rr
import traceback

# Used for sensor fusion
from geometry_msgs.msg import PoseWithCovarianceStamped


class PressureRawPublisher:

    DEPTH_DEST_TOPIC = 'sensors/depth'
    FTDI_FILE_PATH = 'package://data_pub/config/pressure_ftdi.yaml'

    BAUDRATE = 9600
    NODE_NAME = 'pressure_pub'

    def __init__(self):
        with open(rr.get_filename(self.FTDI_FILE_PATH, use_protocol=False)) as f:
            self._ftdi_strings = yaml.safe_load(f)

        self._pub_depth = rospy.Publisher(self.DEPTH_DEST_TOPIC, PoseWithCovarianceStamped, queue_size=50)

        self._current_pressure_msg = PoseWithCovarianceStamped()

        self._serial_port = None
        self._serial = None

    # Read FTDI strings of all ports in list_ports.grep
    def connect(self):
        while self._serial_port is None and not rospy.is_shutdown():
            try:
                self._serial_port = next(list_ports.grep('|'.join(self._ftdi_strings))).device
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
            buff += self._serial.read(1)
        return buff.decode('utf-8', errors='ignore')

    def run(self):
        rospy.init_node(self.NODE_NAME)
        self.connect()
        while not rospy.is_shutdown():
            try:
                # Direct read from device
                line = self.readline_nonblocking().strip()
                if not line or line == '':
                    rospy.logerr("Timeout in pressure serial read, trying again in 2 seconds.")
                    rospy.sleep(0.1)
                    continue  # Skip and retry
                self._pressure = line  # Remove \r\n
                self._parse_pressure()  # Parse pressure data
                self._publish_current_msg()  # Publish pressure data
            except Exception:
                rospy.logerr("Error in reading pressure information. Reconnecting.")
                rospy.logerr(traceback.format_exc())
                self._serial.close()
                self._serial = None
                self._serial_port = None
                self.connect()

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

    def _publish_current_msg(self):
        if abs(self._current_pressure_msg.pose.pose.position.z) > 7:
            self._current_pressure_msg = PoseWithCovarianceStamped()
            return

        self._current_pressure_msg.header.stamp = rospy.Time.now()
        self._current_pressure_msg.header.frame_id = "odom"  # World frame

        self._pub_depth.publish(self._current_pressure_msg)
        self._current_pressure_msg = PoseWithCovarianceStamped()


if __name__ == '__main__':
    try:
        PressureRawPublisher().run()
    except rospy.ROSInterruptException:
        pass
