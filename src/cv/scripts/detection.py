#!/usr/bin/env python

import rospy
from cv.msg import Object
from sensor_msgs.msg import Image


class Detector:

    NODE_NAME = 'cv'
    GATE_TOPIC = '/cv/gate'
    IMAGE_TOPIC = '/test_images/image'

    def __init__(self):
        self.gate_publisher = rospy.Publisher(self.GATE_TOPIC, Object, queue_size=10)

    def detect(self, img_msg):
        object_msg = Object()
        object_msg.xmin = img_msg.height

        self.gate_publisher.publish(object_msg)

    def run(self):
        rospy.init_node(self.NODE_NAME)
        rospy.Subscriber(self.IMAGE_TOPIC, Image, self.detect)

        while not rospy.is_shutdown():
            pass


if __name__ == '__main__':
    Detector().run()
