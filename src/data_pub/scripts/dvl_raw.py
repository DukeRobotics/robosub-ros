#!/usr/bin/env python

import serial
import serial.tools.list_ports as list_ports
import rospy
from std_msgs.msg import String

FTDI_STR = 'FT232R'
BAUDRATE = 9600
TOPIC_NAME = 'dvl_raw'
NODE_NAME = 'dvl_raw_publisher'

def run():
    serial_port = next(list_ports.grep(FTDI_STR)).device
    s = serial.Serial(serial_port, BAUDRATE, 
            timeout=0.1, write_timeout=1.0,
            bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE)
    pub = rospy.Publisher(TOPIC_NAME, String, queue_size=10)
    rospy.init_node(NODE_NAME)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
    	pub.publish(s.readline())
    	rate.sleep()

if __name__ == '__main__':
    try:
    	run()
    except rospy.ROSInterruptException:
    	pass
