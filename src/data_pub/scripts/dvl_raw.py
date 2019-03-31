#!/usr/bin/env python

import serial
import serial.tools.list_ports as list_ports
import rospy
from std_msgs.msg import String
from underwater_sensor_msgs.msg import DVL

class DvlRawPublisher:

    FTDI_STR = 'FT232R'
    BAUDRATE = 9600
    TOPIC_NAME = 'dvl_raw'
    NODE_NAME = 'dvl_raw_publisher'

    def __init__(self):
        self._pub = rospy.Publisher(self.TOPIC_NAME, String, queue_size=10)
        
        self._current_message = DVL()

        self._serial_port = None
        self._serial = None

        self._dvl_line_parsers = {
            'SA': self._parse_SA,
            'TS': self._parse_TS,
            'BI': self._parse_BI,
            'BS': self._parse_BS,
            'BE': self._parse_BE,
            'BD': self._parse_BD
        }


    def run(self):
        rospy.init_node(self.NODE_NAME)

        self._serial_port = next(list_ports.grep(self.FTDI_STR)).device
        self._serial = serial.Serial(self._serial_port, self.BAUDRATE, 
                timeout=0.1, write_timeout=1.0,
                bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE)

        while not rospy.is_shutdown():
            line = self._serial.readline()
            if line.strip() and line[0] == ':':
                self._parse_line(line)

    def _parse_line(self, line):
        rospy.logdebug('Parsing line %s', line)
        data_type = line[1:3]
        self._dvl_line_parsers[data_type](data_type)
        self._pub.publish(line)

    def _parse_SA(self, line):
        pass

    def _parse_TS(self, line):
        pass

    def _parse_BI(self, line):
        pass

    def _parse_BS(self, line):
        pass

    def _parse_BE(self, line):
        pass

    def _parse_BD(self, line):
        pass




if __name__ == '__main__':
    try:
    	DvlRawPublisher().run()
    except rospy.ROSInterruptException:
    	pass
