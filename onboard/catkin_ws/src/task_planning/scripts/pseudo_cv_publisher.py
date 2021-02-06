#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3


class PseudoVision:
    CV_GATE_DATA_TOPIC = 'cv/gate_data'

    def __init__(self):
        rospy.init_node('pseudo_vision_publisher')
        self.gate_data_publisher = rospy.Publisher(self.CV_GATE_DATA_TOPIC, Vector3, queue_size=5)

    def publish_fake_stuff(self):
        rate = rospy.Rate(15)
        gate_vector = Vector3(x=1, y=2, z=3)
        while not rospy.is_shutdown():
            self.gate_data_publisher.publish(gate_vector)
            rate.sleep()


if __name__ == '__main__':
    PseudoVision().publish_fake_stuff()
