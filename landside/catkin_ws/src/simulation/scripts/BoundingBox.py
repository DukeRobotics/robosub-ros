#!/usr/bin/env python
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
import rospy

def get_bounding_box(listener, points):
	#for loop getting grid point for each point
	# cam_pos = [0.309,0.138,0.18]
	# for i in range(3):
	# 	pos[i] += cam_pos[i]
		
	xs = []
	ys = []
	for i in range(len(points)):
		grid_point = get_grid_point(listener, points[i])
		if grid_point != [-1, -1]:
			xs.append(grid_point[0])
			ys.append(grid_point[1])

	bounding_box = get_box(xs, ys)
	print()
	return bounding_box

def get_grid_point(listener, point):
	rel_point = point_rel_to_bot(listener, point)
	#FOV - field of view
	if rel_point[0] < 0:
		print("removing negative point! "+str(point))
		return [-1, -1]
	xFOV = 0.933 * rel_point[0]
	yFOV = 0.586 * rel_point[0]
	xPix = (xFOV / 2 - rel_point[1]) / xFOV
	yPix = (yFOV / 2 - rel_point[2]) / yFOV
	xPix = max(min(1,xPix), 0)
	yPix = max(min(1,yPix), 0)
	grid_point = [xPix, yPix]
	rospy.loginfo("orig point: "+str(point))
	rospy.loginfo("point: "+str(rel_point))
	rospy.loginfo(" grid point: "+str(grid_point))
	rospy.loginfo("")
	# for i in range(2):
	# 	if (grid_point[i] < 0):
	# 		grid_point[i] = 0
	# 	if (grid_point[i] > 1):
	# 		grid_point[i] = 1
	return grid_point

def point_rel_to_bot(listener, point):

	# r = R.from_euler('xyz', orientation, degrees=False)
	# rel_point = r.apply(point)
	#
	# for i in range(len(rel_point)):
	# 	rel_point[i] -= pos[i]
	poses = PoseStamped()
	pose = Pose()
	q = Quaternion()
	q.w = 1
	posepoint = Point()
	posepoint.x = point[0]
	posepoint.y = point[1]
	posepoint.z = point[2]
	pose.position = posepoint
	pose.orientation = q
	poses.pose = pose
	poses.header.frame_id = "odom"
	newpoint = listener.transformPose("cameras_link", poses).pose.position
	rel_point = [0, 0, 0]
	rel_point[0] = newpoint.x
	rel_point[1] = newpoint.y
	rel_point[2] = newpoint.z
	# rospy.loginfo("Before: "+ str(point)+" After: " + str(newpoint))
	return rel_point

def clamp(num, min, max):
	return min(max(min, num), max)

def get_box(xs, ys):
	if len(xs) == 0:
		xs = [-1]
	if len(ys) == 0:
		ys = [-1]
	box = [clamp(min(xs), 0, 1), clamp(max(xs), 0, 1), clamp(min(ys), 0, 1), clamp(max(ys), 0, 1)]
	# if box == [0,1,0,1]:
	# 	box = [-1,-1,-1,-1]
	
	rospy.loginfo("indices {:}, {:}, {:}, {:}".format(xs.index(box[0]), xs.index(box[1]), ys.index(box[2]), ys.index(box[3])))
	# if ((xmin == xmax) or (ymin == ymax)):
	# 	for x in box:
	# 		x = -1
	return box

	#if __name__ == "__main__":
		
