#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def do():
	pub = rospy.Publisher('testtopic', String, queue_size=10)
	rospy.init_node('davids_node')
	rate = rospy.Rate(3)
	while not rospy.is_shutdown():
		pub.publish("Hello!")
		rate.sleep()

if __name__ == '__main__':
	try:
		do()
	except rospy.ROSInterruptException:
		pass
