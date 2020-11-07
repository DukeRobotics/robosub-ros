#!/usr/bin/env python
from scipy.spatial.transform import Rotation as R

def get_bounding_box(points, pos, orientation):
	#for loop getting grid point for each point
	cam_pos = [0.309,0.138,0.18]
	for i in range(3):
		pos[i] += cam_pos[i]
		
	xs = []
	ys = []
	for i in range(len(points)):
		grid_point = get_grid_point(points[i], pos, orientation)
		xs.append(grid_point[0])
		ys.append(grid_point[1])

	bounding_box = get_box(xs, ys)
	return bounding_box

def get_grid_point(point, pos, orientation):
	rel_point = point_rel_to_bot(point, pos, orientation)
	#FOV - field of view
	xFOV = 0.933 * rel_point[0]
	yFOV = 0.586 * rel_point[0]
	xPix = (xFOV / 2 - rel_point[1]) / xFOV
	yPix = (yFOV / 2 - rel_point[2]) / yFOV
	grid_point = [xPix, yPix]
	for i in range(2):
		if (grid_point[i] < 0):
			grid_point[i] = 0
		if (grid_point[i] > 1):
			grid_point[i] = 1
	return grid_point

def point_rel_to_bot(point, pos, orientation):
	r = R.from_euler('xyz', orientation, degrees=True)
	rel_point = r.apply(point)

	for i in range(len(rel_point)):
		rel_point[i] -= pos[i]
	return rel_point

def get_box(xs, ys):
	xmax = 0
	xmin = 1
	ymax = 0
	ymin = 1

	for i in range(len(xs)):
		if (xs[i] > xmax):
			xmax = xs[i]
		if (xs[i] < xmin):
			xmin = xs[i]
		if (ys[i] > ymax):
			ymax = ys[i]
		if (ys[i] < ymin):
			ymin = ys[i]

	box = [xmin, xmax, ymin, ymax]
	if ((xmin == xmax) or (ymin == ymax)):
		for x in box:
			x = -1
	return box

	#if __name__ == "__main__":
		
