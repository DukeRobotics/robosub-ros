########## Bus - bus.py ##########
# Original Author: John Zeller
# Description: Bus initializes the port(s) for use, and offers them up as easy
#	       to access attributes. Additionally, there is the option to reset
#	       the port(s) by using the restart function.

import serial, time

class Bus(object):
        def __init__(self):
                self.gamepad = open('/dev/input/js0', 'r')

        def restart(self):
		self.gamepad.close()
		self.gamepad = open('/dev/input/js0', 'r')
