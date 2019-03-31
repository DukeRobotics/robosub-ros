#!/usr/bin/env python

import serial
import rospy
from std_msgs.msg import String

def do():
    s = serial.Serial('/dev/ttyUSB1', 9600, 
            timeout=0.1, write_timeout=1.0,
            bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE)
    pub = rospy.Publisher('testtopic', String, queue_size=10)
    rospy.init_node('davids_node')
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
    	pub.publish(s.readline())
    	rate.sleep()

if __name__ == '__main__':
    try:
    	do()
    except rospy.ROSInterruptException:
    	pass
