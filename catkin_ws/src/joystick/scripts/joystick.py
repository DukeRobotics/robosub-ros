#!/usr/bin/env python

import threading
import time

import rospy
import numpy as np
import math

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class ParserMain(threading.Thread):
    NODE_NAME = 'joy_pub'
    JOYSTICK_RAW_TOPIC = 'joystick/raw'
    JOY_DEST_TOPIC_LOCAL = 'controls/desired_twist_power'

    def __init__(self):
	self._pub_joy_local = rospy.Publisher(self.JOY_DEST_TOPIC_LOCAL, Twist, queue_size=50)
	rospy.init_node(self.NODE_NAME)
	self._current_joy_local_msg = Twist()
	
	self._movement_type = 0		# Where 0 defines the left joystick to translational movement (x, y)
				        #   and 1 defines the left joystick to rotation (roll, pitch)
        rospy.Subscriber(self.JOYSTICK_RAW_TOPIC, Joy, self.parse_data)
	rospy.spin()
       	
	
    def parse_data(self, raw_joystick_data):
        self._initialize_joystick_data(raw_joystick_data)       

	if (self._movement_type == 0): 
            self._parse_local_linear()
	else:
            self._parse_local_angular()
	self._publish_current_msg()

    def _parse_local_linear(self):
	self._current_joy_local_msg.linear.x = self._leftLR
	self._current_joy_local_msg.linear.y = self._leftUD
	self._current_joy_local_msg.linear.z = self._rightUD

	self._current_joy_local_msg.angular.x = 0			
	self._current_joy_local_msg.angular.y = 0			
	self._current_joy_local_msg.angular.z = self._rightLR

    def _parse_local_angular(self):
	self._current_joy_local_msg.linear.x = 0			
	self._current_joy_local_msg.linear.y = 0
	self._current_joy_local_msg.linear.z = self._rightUD

	self._current_joy_local_msg.angular.x = self._leftLR
       	self._current_joy_local_msg.angular.y = self._leftUD				
	self._current_joy_local_msg.angular.z = self._rightLR

    def _initialize_joystick_data(self, raw_joystick_data):

        self._leftLR = raw_joystick_data.axes[0]
	self._leftUD = raw_joystick_data.axes[1]
	self._rightLR = raw_joystick_data.axes[2]
	self._rightUD = raw_joystick_data.axes[3]
		
        if (raw_joystick_data.buttons[2] == 1):
            self._movement_type = 0

        if (raw_joystick_data.buttons[3] == 1):
    	    self._movement_type = 1		


    def _publish_current_msg(self):
        self._pub_joy_local.publish(self._current_joy_local_msg)
        self._current_joy_local_msg = Twist()
				
if __name__ == '__main__':
    try:
    	parser = ParserMain()
    except rospy.ROSInterruptException:
        pass

