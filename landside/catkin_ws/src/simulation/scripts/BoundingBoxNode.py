#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
from custom_msgs.msg import CVObject
from tf import TransformListener

import BoundingBox

pub_gate = rospy.Publisher('/gate/left', CVObject, queue_size=10)
pub_buoy = rospy.Publisher('/buoy/left', CVObject, queue_size=10)
listener = None

def node_code():
	global listener
	rospy.loginfo("running!")
	rospy.init_node('box_maker')
	listener = TransformListener()
	rospy.Subscriber("/sim/object_points", Float32MultiArray, callback)
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		hello_str = "hello world %s" % rospy.get_time()
		#rospy.loginfo(hello_str)
		rate.sleep()


def callback(data):
	global pub_gate, pub_buoy

	# rospy.loginfo("object points received")

	objPoints = parse_array(data.data)
	print("objpoints:", objPoints)
	boxes = bounding_boxes(objPoints)
	print("boxes:", boxes)

	# rospy.loginfo(objects)

	for i in boxes:
		box = CVObject()
		box.score = 1

		box.xmin = boxes[i][0]
		box.xmax = boxes[i][1]
		box.ymin = boxes[i][2]
		box.ymax = boxes[i][3]
		if i == 1: #gate==1
			box.label = 'gate'
			pub_gate.publish(box)
		if i == 2: #buoy==2
			box.label = 'buoy'
			pub_buoy.publish(box)

#parse into arrays of arrays of points
def parse_array(array):
	ret = {}
	print(array)
	print(len(array))
	while array:
		if len(array) < 2:
			break
		obj = array[0]
		if obj not in ret:
			ret[obj] = []
		numPoints = array[1]
		array = array[2:]
		for i in range(int(numPoints)):
			ret[obj].append(array[:3])
			array = array[3:]
	return ret


def bounding_boxes(objPoints):
	boxes = {}
	for i in objPoints:
		boxes[i] = BoundingBox.get_bounding_box(listener, objPoints[i])
	return boxes

if __name__ == "__main__":
	rospy.loginfo("Bounding Box Node running")
	node_code()
