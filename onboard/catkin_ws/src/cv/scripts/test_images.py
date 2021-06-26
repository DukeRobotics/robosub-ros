#!/usr/bin/env python3

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
    CAMERA_2 = 'right'
    IMAGE_TOPIC = '/camera/{}/image_raw'.format(CAMERA)
    IMAGE_TOPIC_2 = '/camera/{}/image_raw'.format(CAMERA_2)

    # Toggle these options to custom dummy images and enable stereo mode
    IMAGE = '../assets/left0.jpg'
    IMAGE_2 = '../assets/right0.jpg'
    STEREO_ENABLED = True

    # Read in the dummy image and other misc. setup work
    def __init__(self):
        self.image_publisher = rospy.Publisher(self.IMAGE_TOPIC, Image, queue_size=10)
        self.image_publisher_2 = rospy.Publisher(self.IMAGE_TOPIC_2, Image, queue_size=10)

        path = os.path.dirname(__file__)
        image = cv2.imread(os.path.join(path, self.IMAGE, cv2.IMREAD_COLOR))
        image_2 = cv2.imread(os.path.join(path, self.IMAGE_2, cv2.IMREAD_COLOR))
        bridge = CvBridge()

        self.image_msg = bridge.cv2_to_imgmsg(image, 'bgr8')
        self.image_msg_2 = bridge.cv2_to_imgmsg(image_2, 'bgr8')

    # Publish dummy image to topic every few seconds
    def run(self):
        rospy.init_node(self.NODE_NAME)

        # Testing enable_model service
        service_name = 'enable_model_{}'.format(self.CAMERA if not self.STEREO_ENABLED else 'stereo')

        rospy.wait_for_service(service_name)
        enable_model = rospy.ServiceProxy(service_name, EnableModel)

        loop_rate = rospy.Rate(1)

        # Logic goes below to simulate publishing of camera frames and toggling of models:
        model_enabled = True
        count = 0

        while not rospy.is_shutdown():
            self.image_publisher.publish(self.image_msg)
            self.image_publisher_2.publish(self.image_msg_2)

            if count % 30 == 0:
                enable_model('default', model_enabled)
                model_enabled = not model_enabled

            count += 1
            loop_rate.sleep()


if __name__ == '__main__':
    DummyImagePublisher().run()
