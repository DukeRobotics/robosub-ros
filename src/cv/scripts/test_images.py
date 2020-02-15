#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image


class DummyImagePublisher:

    NODE_NAME = 'test_images'
    IMAGE_TOPIC = '/test_images/image'

    def __init__(self):
        # image_topic_name = '{}{}'.format(self.NODE_NAME, self.IMAGE_TOPIC)
        self.image_publisher = rospy.Publisher(self.IMAGE_TOPIC, Image, queue_size=10)

        self.image_msg = Image()
        self.image_msg.height = 100

    def run(self):
        rospy.init_node(self.NODE_NAME)

        loop_rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.image_publisher.publish(self.image_msg)
            loop_rate.sleep()


if __name__ == '__main__':
    DummyImagePublisher().run()
