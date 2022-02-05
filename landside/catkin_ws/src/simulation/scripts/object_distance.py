#!/usr/bin/env python

from custom_msgs.msg import SimObjectArray
from tf import TransformListener
import rospy
from numpy import clip
from os import path
import resource_retriever as rr
import yaml
from std_msgs.msg import Float64

class ObjectDistance:

    def __init__(self):
        rospy.init_node('sim_object_distance_tracker')
        self.listener = TransformListener()
        config_filepath = rr.get_filename(
            'package://simulation/data/config.yaml', 
            use_protocol=False
        )
        with open(config_filepath) as f:
            data = yaml.safe_load(f)
        self.publishers = {i.strip(): rospy.Publisher(f'/{i.strip().lower()}/distance', Float64, queue_size=10)
                           for i in data['cv_objects']}
        rospy.Subscriber("/sim/object_points", SimObjectArray, self.callback)
        rospy.spin()

    def callback(self, data):
        distances = {obj.label: obj.distance for obj in data.objects}

        for label in distances:
            distance_msg = Float64()
            distance_msg.data = distances[label]
            # box.label = label
            # if label not in self.publishers:
            #     rospy.logerr(f"bounding_box.callback: Publisher for label {label} not found")
            #     continue
            self.publishers[label].publish(distance_msg)

if __name__ == "__main__":
    ObjectDistance()
