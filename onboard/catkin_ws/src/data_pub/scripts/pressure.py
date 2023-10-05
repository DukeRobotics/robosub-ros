#!/usr/bin/env python3

import rospy
import serial
import serial.tools.list_ports as list_ports
import yaml
import os
import resource_retriever as rr
import traceback

from geometry_msgs.msg import PoseWithCovarianceStamped

class PressureRawPublisher:

    DEPTH_DEST_TOPIC = 'sensors/pressure'
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

    #read FTDI strings of all ports in list_ports.grep

    def connect(self):
        while self._serial_port is None and not rospy.is_shutdown():
            try:
                #self._serial_port = next(list_ports.grep('|'.join(self._ftdi_strings))).device
                self._serial_port = "/dev/ttyACM0"
                self._serial = serial.Serial(self._serial_port, self.BAUDRATE,
                                             timeout=None, write_timeout=None,
                                             bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                                             stopbits=serial.STOPBITS_ONE)
            except StopIteration:
                rospy.logerr("IMU not found, trying again in 0.1 seconds.")
                rospy.sleep(0.1)

    def run(self):
        rospy.init_node(self.NODE_NAME)
        self.connect()
        while not rospy.is_shutdown():
            try:
                line = self._serial.readline().decode('utf-8')
                self._pressure = line[:-2]
                self._parse_pressure()
                self._publish_current_msg()
            except Exception:
                rospy.logerr("Error in reading and extracting information. Reconnecting.")
                rospy.logerr(traceback.format_exc())
                self._serial.close()
                self._serial = None
                self._serial_port = None
                self.connect()

    def _parse_pressure(self):
        self._current_pressure_msg.pose.pose.position.x = 0.0
        self._current_pressure_msg.pose.pose.position.y = 0.0
        self._current_pressure_msg.pose.pose.position.z = -1* float(self._pressure)

        self._current_pressure_msg.pose.pose.orientation.x = 0.0
        self._current_pressure_msg.pose.pose.orientation.y = 0.0
        self._current_pressure_msg.pose.pose.orientation.z = 0.0
        self._current_pressure_msg.pose.pose.orientation.w = 1.0

        self._current_pressure_msg.pose.covariance[14] = 0.01

    def _publish_current_msg(self):
        self._current_pressure_msg.header.stamp = rospy.Time.now()
        self._current_pressure_msg.header.frame_id = "odom"

        self._pub_depth.publish(self._current_pressure_msg)
        self._current_pressure_msg = PoseWithCovarianceStamped()


if __name__ == '__main__':
    try:
        PressureRawPublisher().run()
    except rospy.ROSInterruptException:
        pass
