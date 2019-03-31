#!/usr/bin/env python

import serial
import serial.tools.list_ports as list_ports
import rospy
from std_msgs.msg import String
from underwater_sensor_messages.msg import DVL

class DvlRawPublisher:

    FTDI_STR = 'FT232R'
    BAUDRATE = 9600
    TOPIC_NAME = 'dvl_raw'
    NODE_NAME = 'dvl_raw_publisher'

    def __init__(self):
        self.current_message = DVL()

        self._serial_port = None
        self._serial = None

    def run():
        self._serial_port = next(list_ports.grep(FTDI_STR)).device
        self._serial = serial.Serial(self._serial_port, BAUDRATE, 
                timeout=0.1, write_timeout=1.0,
                bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE)
        pub = rospy.Publisher(TOPIC_NAME, String, queue_size=10)
        rospy.init_node(NODE_NAME)
        #rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            parse_line(
            #rate.sleep()

if __name__ == '__main__':
    try:
    	run()
    except rospy.ROSInterruptException:
    	pass
