#!/usr/bin/env python

from custom_msgs.msg import CVObject, SimObjectArray
from tf import TransformListener
from geometry_msgs.msg import Pose, Quaternion, PoseStamped
import rospy
from numpy import clip


class BoundingBox:

    def __init__(self):
        rospy.init_node('sim_box_maker')
        self.listener = TransformListener()
<<<<<<< HEAD
        self.publishers = {'gate': rospy.Publisher('/gate/left', CVObject, queue_size=10),
                           'buoy': rospy.Publisher('/buoy/left', CVObject, queue_size=10)}
        rospy.Subscriber("/sim/object_points", SimObjectArray, self.callback)
        rospy.spin()

    def callback(self, data):
        boxes = {obj.label: self.get_bounding_box(obj.points) for obj in data.objects}
=======
        self.pub_gate = rospy.Publisher('/gate/left', CVObject, queue_size=10)
        self.pub_buoy = rospy.Publisher('/buoy/left', CVObject, queue_size=10)
        rospy.Subscriber("/sim/object_points", Float32MultiArray, self.callback)
        rospy.spin()

    def callback(self, data):
        boxes = self.bounding_boxes(self.parse_array(data.data))
>>>>>>> add_simulation_objs

        for label in boxes:
            box = CVObject()
            box.score = 1
<<<<<<< HEAD
            box.label = label
            box.xmin, box.xmax, box.ymin, box.ymax = boxes[label]
            if label not in self.publishers:
                rospy.logerr(f"Publisher for label {label} not found")
                continue
            self.publishers[label].publish(box)

    def get_bounding_box(self, points):
        grid_points = [self.get_grid_point(point) for point in points]
        filtered_grid_points = [point for point in grid_points if point != (-1, -1)]
        xs, ys = zip(*(filtered_grid_points if filtered_grid_points else [(-1, -1)]))
        rospy.loginfo(f"{xs}, {ys}")
        return (clip(min(xs), 0, 1), clip(max(xs), 0, 1), clip(min(ys), 0, 1), clip(max(ys), 0, 1))
=======

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
>>>>>>> add_simulation_objs

    def get_grid_point(self, point):
        rel_point = self.point_rel_to_bot(point)
        # FOV - field of view
<<<<<<< HEAD
        if rel_point.x < 0:
            return (-1, -1)
        xFOV = 0.933 * rel_point.x
        yFOV = 0.586 * rel_point.x
        xPix = (xFOV / 2 - rel_point.y) / xFOV
        yPix = (yFOV / 2 - rel_point.z) / yFOV
        xPix = max(min(1, xPix), 0)
        yPix = max(min(1, yPix), 0)
        return (xPix, yPix)

    def point_rel_to_bot(self, point):
        poses = PoseStamped()
        poses.pose = Pose(position=point, orientation=Quaternion(w=1, x=0, y=0, z=0))
        poses.header.frame_id = "odom"
        transformed = self.listener.transformPose("cameras_link", poses).pose.position
        return transformed


if __name__ == "__main__":
=======
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
>>>>>>> add_simulation_objs
    BoundingBox()
