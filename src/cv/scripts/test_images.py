#!/usr/bin/env python

import rospy
import cv2
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


# Mock the camera by publishing the same image to a topic
class DummyImagePublisher:

    NODE_NAME = 'test_images'
    IMAGE_TOPIC = '/test_images/image'

    # Read in the dummy image and other misc. setup work
    def __init__(self):
        self.image_publisher = rospy.Publisher(self.IMAGE_TOPIC, Image, queue_size=10)

        path = os.path.dirname(__file__)
        image = cv2.imread(os.path.join(path, '../assets/buoy.jpg'), cv2.IMREAD_COLOR)
        bridge = CvBridge()

        self.image_msg = bridge.cv2_to_imgmsg(image, 'bgr8')

    # Publish dummy image to topic every 5 seconds
    def run(self):
        rospy.init_node(self.NODE_NAME)

        loop_rate = rospy.Rate(0.2)
        while not rospy.is_shutdown():
            self.image_publisher.publish(self.image_msg)
            loop_rate.sleep()


if __name__ == '__main__':
    DummyImagePublisher().run()
