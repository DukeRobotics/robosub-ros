#!/usr/bin/env python

from custom_msgs.msg import CVObject, SimObjectArray
from tf import TransformListener
from geometry_msgs.msg import Pose, Quaternion, PoseStamped
import rospy
from numpy import clip
from os import path
import resource_retriever as rr
import yaml

class BoundingBox:

    def __init__(self):
        rospy.init_node('sim_box_maker')
        self.listener = TransformListener()
        config_filepath = rr.get_filename(
            'package://simulation/data/config.yaml', 
            use_protocol=False
        )
        with open(config_filepath) as f:
            data = yaml.safe_load(f)
        self.publishers = {i.strip(): rospy.Publisher(f'/{i.strip().lower()}/left', CVObject, queue_size=10)
                           for i in data['cv_objects']}
        rospy.Subscriber("/sim/object_points", SimObjectArray, self.callback)
        rospy.spin()

    def callback(self, data):
        boxes = {obj.label: self.get_bounding_box(obj.points) for obj in data.objects}

        for label in boxes:
            box = CVObject()
            box.score = 1
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
        return clip(min(xs), 0, 1), clip(max(xs), 0, 1), clip(min(ys), 0, 1), clip(max(ys), 0, 1)

    def get_grid_point(self, point):
        rel_point = self.point_rel_to_bot(point)
        # FOV - field of view
        if rel_point.x < 0:
            return (-1, -1)
        xFOV = 0.933 * rel_point.x
        yFOV = 0.586 * rel_point.x
        xPix = (xFOV / 2 - rel_point.y) / xFOV
        yPix = (yFOV / 2 - rel_point.z) / yFOV
        return clip(xPix, 0, 1), clip(yPix, 0, 1)

    def point_rel_to_bot(self, point):
        poses = PoseStamped()
        poses.pose = Pose(position=point, orientation=Quaternion(w=1, x=0, y=0, z=0))
        poses.header.frame_id = "odom"
        transformed = self.listener.transformPose("cameras_link", poses).pose.position
        return transformed


if __name__ == "__main__":
    BoundingBox()
