#!/usr/bin/env python

import threading
from bus import *
from parser_core import *
import time

import rospy
import numpy as np
import math

from geometry_msgs.msg import Twist

class ParserMain(threading.Thread):
    NODE_NAME = 'joy_pub'
    JOY_DEST_TOPIC_LOCAL = 'controls/desired_twist_power'

    def __init__(self):
	self._pub_joy_local = rospy.Publisher(self.JOY_DEST_TOPIC_LOCAL, Twist, queue_size=50)
       
       	self._current_joy_local_msg = Twist()
	
	self._movement_type = 0		# Where 0 defines the left joystick to translational movement (x, y)
				        #   and 1 defines the left joystick to rotation (roll, pitch)
		
	
	# Create bus object
	self.bus = Bus()
	# Create a dictionary to be used to keep states from joy_core
	self.states = { 'A':0, 'B':0, 'X':0, 'Y':0,  		   		\
			'Back':0, 'Start':0, 'Middle':0,		   	\
			'Left':0, 'Right':0, 'Up':0, 'Down':0, 			\
			'LB':0, 'RB':0, 'LT':0, 'RT':0,				\
			'LJ/Button':0, 'RJ/Button':0, 				\
			'LJ/Left':0, 'LJ/Right':0, 'LJ/Up':0, 'LJ/Down':0, 	\
			'RJ/Left':0, 'RJ/Right':0, 'RJ/Up':0, 'RJ/Down':0}	

        self._leftLR = (self.states['LJ/Left'] + self.states['LJ/Right']) / 128.0
	self._leftUD = (self.states['LJ/Up'] + self.states['LJ/Down']) / 128.0
	self._rightLR = (self.states['RJ/Left'] + self.states['RJ/Right']) / 128.0
	self._rightUD = (self.states['RJ/Up'] + self.states['RJ/Down']) / 128.0
	
	# Launch Parser_Core as a seperate thread to parse the gamepad
	self.parsercore = ParserCore(self.bus, self.states)
	self.parsercore.start()
	
    def run(self):
        rospy.init_node(self.NODE_NAME)
	    
        while not rospy.is_shutdown():
            rospy.loginfo(self._leftLR)
            self._initialize_joystick_data()       

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

    def _initialize_joystick_data(self):

        self._leftLR = (self.states['LJ/Left'] + self.states['LJ/Right']) / 128.0
	self._leftUD = (self.states['LJ/Up'] + self.states['LJ/Down']) / 128.0
	self._rightLR = (self.states['RJ/Left'] + self.states['RJ/Right']) / 128.0
	self._rightUD = (self.states['RJ/Up'] + self.states['RJ/Down']) / 128.0
		
        if (self.states['X'] == 1):
            self._movement_type = 0

        if (self.states['Y'] == 1):
    	    self._movement_type = 1		


    def _publish_current_msg(self):
        self._pub_joy_local.publish(self._current_joy_local_msg)
        self._current_joy_local_msg = Twist()
				
if __name__ == '__main__':
    try:
    	parser = ParserMain()
       	parser.run()
    except rospy.ROSInterruptException:
        pass

