#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
from custom_msgs.msg import CVObject
from tf import TransformListener
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
import rospy
from numpy import clip


class BoundingBox:

    def __init__(self):
        rospy.loginfo("running!")
        rospy.init_node('box_maker')
        self.listener = TransformListener()
        self.pub_gate = rospy.Publisher('/gate/left', CVObject, queue_size=10)
        self.pub_buoy = rospy.Publisher('/buoy/left', CVObject, queue_size=10)

    def run(self):
        rospy.Subscriber("/sim/object_points", Float32MultiArray, self.callback)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            hello_str = "hello world %s" % rospy.get_time()
            # rospy.loginfo(hello_str)
            rate.sleep()

    def callback(self, data):
        objPoints = self.parse_array(data.data)
        boxes = self.bounding_boxes(objPoints)

        for i in boxes:
            box = CVObject()
            box.score = 1

            box.xmin = boxes[i][0]
            box.xmax = boxes[i][1]
            box.ymin = boxes[i][2]
            box.ymax = boxes[i][3]
            if i == 1:  # gate==1
                box.label = 'gate'
                self.pub_gate.publish(box)
            if i == 2:  # buoy==2
                box.label = 'buoy'
                self.pub_buoy.publish(box)

    # parse into arrays of arrays of points
    def parse_array(self, array):
        ret = {}
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

    def bounding_boxes(self, objPoints):
        boxes = {}
        for i in objPoints:
            boxes[i] = self.get_bounding_box(objPoints[i])
        return boxes

    def get_bounding_box(self, points):
        # for loop getting grid point for each point
        # cam_pos = [0.309,0.138,0.18]
        # for i in range(3):
        # 	pos[i] += cam_pos[i]

        xs = []
        ys = []
        for i in range(len(points)):
            grid_point = self.get_grid_point(points[i])
            if grid_point != [-1, -1]:
                xs.append(grid_point[0])
                ys.append(grid_point[1])

        bounding_box = self.get_box(xs, ys)
        print()
        return bounding_box

    def get_grid_point(self, point):
        rel_point = self.point_rel_to_bot(point)
        # FOV - field of view
        if rel_point[0] < 0:
            print("removing negative point! " + str(point))
            return [-1, -1]
        xFOV = 0.933 * rel_point[0]
        yFOV = 0.586 * rel_point[0]
        xPix = (xFOV / 2 - rel_point[1]) / xFOV
        yPix = (yFOV / 2 - rel_point[2]) / yFOV
        xPix = max(min(1, xPix), 0)
        yPix = max(min(1, yPix), 0)
        grid_point = [xPix, yPix]
        rospy.loginfo("orig point: " + str(point))
        rospy.loginfo("point: " + str(rel_point))
        rospy.loginfo(" grid point: " + str(grid_point))
        rospy.loginfo("")
        # for i in range(2):
        # 	if (grid_point[i] < 0):
        # 		grid_point[i] = 0
        # 	if (grid_point[i] > 1):
        # 		grid_point[i] = 1
        return grid_point

    def point_rel_to_bot(self, point):
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
        newpoint = self.listener.transformPose("cameras_link", poses).pose.position
        # the above *should* be cameras_link, probably, but that gives weird transforms
        rel_point = [0, 0, 0]
        rel_point[0] = newpoint.x
        rel_point[1] = newpoint.y
        rel_point[2] = newpoint.z
        rospy.loginfo("Before: " + str(point) + " After: " + str(newpoint))
        return rel_point

    def get_box(self, xs, ys):
        print(xs, ys)
        if len(xs) == 0:
            xs = [-1]
        if len(ys) == 0:
            ys = [-1]
        box = [clip(min(xs), 0, 1), clip(max(xs), 0, 1), clip(min(ys), 0, 1), clip(max(ys), 0, 1)]
        # if box == [0,1,0,1]:
        # 	box = [-1,-1,-1,-1]

        # rospy.loginfo("indices {:}, {:}, {:}, {:}".format(xs.index(box[0]), xs.index(box[1]), ys.index(box[2]), ys.index(box[3])))
        # if ((xmin == xmax) or (ymin == ymax)):
        # 	for x in box:
        # 		x = -1
        return box


if __name__ == "__main__":
    rospy.loginfo("Bounding Box Node running")
    BoundingBox().run()
