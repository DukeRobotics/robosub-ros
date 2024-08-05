#!/usr/bin/env python3

from abc import ABC, abstractmethod
import rospy
import time
import serial
import serial.tools.list_ports as list_ports
import yaml
import resource_retriever as rr
import traceback


class SerialPublisher(ABC):
    """
    Abstract serial publisher for a ROS node
    """

    def __init__(self, node_name, baud, config_file_path, config_name):
        """
        @param node_name: The ROS node name
        @param baud: the baud rate
        @config_file_path: location of serial config
        @config_name: name of Arduino to reference from config file
        """

        self.config_file_path = config_file_path
        self.baud = baud
        self.node_name = node_name
        self.config_name = config_name

        with open(rr.get_filename(self.config_file_path, use_protocol=False)) as f:
            config_data = yaml.safe_load(f)
            self._arduino_config = config_data['arduino']

    def connect(self):
        """
        Read FTDI strings of all ports in list_ports.grep
        """
        while self._serial_port is None and not rospy.is_shutdown():
            try:
                pressure_ftdi_string = self._arduino_config[self.config_name]['ftdi']
                print(f"Looking for {self.config_name} sensor with FTDI string {pressure_ftdi_string}.")
                self._serial_port = next(list_ports.grep(pressure_ftdi_string)).device
                print(f"{self.config_name} sensor found at {self._serial_port}.")
                self._serial = serial.Serial(self._serial_port, self.baud,
                                             timeout=1, write_timeout=None,
                                             bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                                             stopbits=serial.STOPBITS_ONE)
            except StopIteration:
                rospy.logerr(f"{self.config_name} sensor not found, trying again in 0.1 seconds.")
                rospy.sleep(0.1)

    def readline_nonblocking(self, tout=1):
        """
        Read line from serial port without blocking

        @param tout: timeout, default = 1 sec
        """
        start = time.time()
        buff = b''
        while ((time.time() - start) < tout) and (b'\r\n' not in buff):
            try:
                buff += self._serial.read(1)
            except serial.SerialException:
                pass

        return buff.decode('utf-8', errors='ignore')

    def writeline(self, line):
        """
        Write line to serial port

        @param line: the line to write
        """
        self._serial.write(line.encode('utf-8') + b'\r\n')

    @abstractmethod
    def process_line(self, line):
        """
        Abstract method to implement how serial input is formatted.

        @param line: the line to be processed
        """
        pass

    def run(self):
        """
        Runs the serial publisher

        Initializes ROS node
        Connects to serial device
        Processes and publishes the serial data to ROS
        """

        rospy.init_node(self.node_name)
        self.connect()

        while not rospy.is_shutdown():
            try:
                # Direct read from device
                line = self.readline_nonblocking().strip()  # Example: "P: 0.22"

                if not line or line == '':
                    rospy.logerr(f"Timeout in {self.config_name} serial read, trying again in 1 second.")
                    rospy.sleep(0.1)
                    continue  # Skip and retry

                self.process_line(line)

            except Exception:
                rospy.logerr(f"Error in reading {self.config_name} serial read, trying again in 1 second.")
                rospy.logerr(traceback.format_exc())
                self._serial.close()
                self._serial = None
                self._serial_port = None
                self.connect()
