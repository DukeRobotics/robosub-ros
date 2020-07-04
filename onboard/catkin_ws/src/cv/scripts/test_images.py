#!/usr/bin/env python

import rospy
import cv2
import os
from custom_msgs.srv import EnableModel
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


# Mock the camera by publishing the same image to a topic
class DummyImagePublisher:

    NODE_NAME = 'test_images'
    CAMERA = 'left'
    IMAGE_TOPIC = '/camera/{}/image_raw'.format(CAMERA)

    # Read in the dummy image and other misc. setup work
    def __init__(self):
        self.image_publisher = rospy.Publisher(self.IMAGE_TOPIC, Image, queue_size=10)

        path = os.path.dirname(__file__)
        image = cv2.imread(os.path.join(path, '../assets/buoy.jpg'), cv2.IMREAD_COLOR)
        bridge = CvBridge()

        self.image_msg = bridge.cv2_to_imgmsg(image, 'bgr8')

    # Publish dummy image to topic every few seconds
    def run(self):
        rospy.init_node(self.NODE_NAME)

        # Testing enable_model service
        service_name = 'enable_model_{}'.format(self.CAMERA)
        rospy.wait_for_service(service_name)
        enable_model = rospy.ServiceProxy(service_name, EnableModel)

        loop_rate = rospy.Rate(1)
        model_enabled = True

        count = 0
        while not rospy.is_shutdown():
            self.image_publisher.publish(self.image_msg)

            # Testing enable
            if count % 30 == 0:
                enable_model('buoy', model_enabled)
                model_enabled = not model_enabled

            count += 1
            loop_rate.sleep()


if __name__ == '__main__':
    DummyImagePublisher().run()
