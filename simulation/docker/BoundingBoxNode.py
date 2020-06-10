#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
import tf

pos = [0,0,0]
orientation = [0,0,0]

def node_code():
	rospy.init_node('box_maker', anonymous=True)	
	rospy.Subscriber("/sim/object_points", Float32MultiArray, callback)
	rospy.Subscriber("/sim/pose", PoseStamped, pose_callback)	
	rate = rospy.Rate(10)

def pose_callback(data):
	global pos
	global orientation

	position = data.pose.position
	pos[0] = position.x
	pos[1] = position.y
	pos[2] = position.z

	(r, p, y) = tf.transformations.euler_from_quaternion([data.pose.orientation.x, 
		data.pose.orientation.y, 
		data.pose.orientation.z, 
		data.pose.orientation.w])
	orientation[0] = r
	orientation[1] = p
	orientation[2] = y

def callback(data):
	global pos
	global orientation

	points, objects = parse_array(data.data)
	boxes = bounding_boxes(points, pos, orientation)
	pub_gate = rospy.Publisher('GateData',Float32MultiArray)
	pub_buoy = rospy.Publisher('BuoyData',Float32MultiArray)	

	for i in range(len(objects)):
		if (objects[i] == 1):
			box = Float32MultiArray()
			box.data = boxes[i]
			pub_gate.publish(box)
		if (objects[i] == 2):
			box = Float32MultiArray()
			box.data = boxes[i]
			pub_buoy.publish(box)

#parse into arrays of arrays of points
def parse_array(array):
	x = 0
	y = 0
	i = 0
	objects_count = 0
	all_points = [[[]]]
	objects = []
	while (i < len(array)):
		objects[objects_count] = array[i]
		objects_count = objects_count + 1
		y = 0
		i = i + 1
		while (i != -999999):
			for j in range(3):
				all_points[x][y][j] = array[i]
				i = i + 1
			y = y + 1
		i = i + 1
		x = x + 1
	return all_points, objects

def bounding_boxes(all_points, pos, orientation):
	boxes = [[]]
	for i in range(len(all_points)):
		boxes[i] = BoundingBox.get_bounding_box(all_points[i], pos, orientation)
	return boxes

