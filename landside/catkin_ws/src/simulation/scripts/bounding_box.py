#!/usr/bin/env python

from std_msgs.msg import Float32MultiArray
from custom_msgs.msg import CVObject
from tf import TransformListener
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
import rospy
from numpy import clip


class BoundingBox:

    def __init__(self):
        rospy.init_node('sim_box_maker')
        self.listener = TransformListener()
        self.pub_gate = rospy.Publisher('/gate/left', CVObject, queue_size=10)
        self.pub_buoy = rospy.Publisher('/buoy/left', CVObject, queue_size=10)
        rospy.Subscriber("/sim/object_points", Float32MultiArray, self.callback)
        rospy.spin()

    def callback(self, data):
        boxes = self.bounding_boxes(self.parse_array(data.data))

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
        return {i: self.get_bounding_box(objPoints[i]) for i in objPoints}

    def get_bounding_box(self, points):
        zipped = [tuple(self.get_grid_point(i)) for i in points]
        xs, ys = tuple(zip(*[i for i in zipped if i != (-1, -1)]))

        return self.get_box(xs, ys)

    def get_grid_point(self, point):
        rel_point = self.point_rel_to_bot(point)
        # FOV - field of view
        if rel_point[0] < 0: # removing point with negative x (behind the robot)
            return [-1, -1]
        xFOV = 0.933 * rel_point[0]
        yFOV = 0.586 * rel_point[0]
        xPix = clip((xFOV / 2 - rel_point[1]) / xFOV, 0, 1)
        yPix = clip((yFOV / 2 - rel_point[2]) / yFOV, 0, 1)
        return [xPix, yPix]

    def point_rel_to_bot(self, point):
        poses = PoseStamped()
        pose = Pose()
        pose.position = Point()
        pose.position.x = point[0]
        pose.position.y = point[1]
        pose.position.z = point[2]
        pose.orientation = Quaternion()
        pose.orientation.w = 1
        poses.pose = pose
        poses.header.frame_id = "odom"
        newpoint = self.listener.transformPose("cameras_link", poses).pose.position
        return [newpoint.x, newpoint.y, newpoint.z]

    def get_box(self, xs, ys):
        x = xs if len(xs) > 0 else [-1]
        y = ys if len(ys) > 0 else [-1]
        return [clip(min(x), 0, 1), clip(max(x), 0, 1), clip(min(y), 0, 1), clip(max(y), 0, 1)]


if __name__ == "__main__":
    rospy.loginfo("Bounding Box Node running")
    BoundingBox()
