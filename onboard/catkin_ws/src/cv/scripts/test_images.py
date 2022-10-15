#!/usr/bin/env python3

import rospy
import cv2
import os
from custom_msgs.srv import EnableModel
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class DummyImagePublisher:
    """Mock the camera by publishing a still image to a topic."""

    NODE_NAME = 'test_images'
    CAMERA = 'left'
    IMAGE_TOPIC = f'/camera/{CAMERA}/image_raw'

    def __init__(self):
        """Read in the dummy image and other misc. setup work."""
        self.image_publisher = rospy.Publisher(self.IMAGE_TOPIC, Image,
                                               queue_size=10)

        path = os.path.dirname(__file__)
        image = cv2.imread(os.path.join(path, '../assets/left384.jpg'),
                           cv2.IMREAD_COLOR)
        bridge = CvBridge()

        self.image_msg = bridge.cv2_to_imgmsg(image, 'bgr8')

    def run(self):
        """Publish dummy image to topic every few seconds.

        Will wait until the enable_model_<camera> service becomes available before starting to publish.
        Every 30 frames published, this node will toggle the enable for the prediction model to test the service.
        """
        rospy.init_node(self.NODE_NAME)

        # Testing enable_model service
        service_name = f'enable_model_{self.CAMERA}'
        rospy.wait_for_service(service_name)
        enable_model = rospy.ServiceProxy(service_name, EnableModel)

        loop_rate = rospy.Rate(1)
        model_enabled = True

        count = 0
        while not rospy.is_shutdown():
            self.image_publisher.publish(self.image_msg)

            # Testing enable
            if count % 30 == 0:
                enable_model('gate', model_enabled)
                model_enabled = not model_enabled

            count += 1
            loop_rate.sleep()

    # TODO: Complete this function
    # Assume that self.feed_path is a string with the path to a rosbag file
    def run_bag(self):
        """Publish a simulated image feed from a rosbag file to a topic.

        Once it publishes all images in the rosbag file, it loops and publishes images from the beginning again.
        """

    # TODO: Complete this function
    # Assume that self.feed_path is a string with the path to a AVI video file
    def run_avi(self):
        """Publish a simulated image feed from a AVI video file to a topic.

        Once it publishes all images in the AVI file, it loops and publishes images from the beginning again.
        """

    # TODO: Complete this function
    # Assume that self.feed_path is a string with the path to a MOV video file
    def run_avi(self):
        """Publish a simulated image feed from a MOV video file to a topic.

        Once it publishes all images in the MOV file, it loops and publishes images from the beginning again.
        """
    
    # TODO: Complete this function
    # Assume that self.feed_path is a string with the path to a MOV video file
    def run_avi(self):
        """Publish a simulated image feed from a folder containing images to a topic.

        All JPG images in the folder will be published in alphabetical order by filename.
        Once it publishes all JPG images in the folder, it loops and publishes images from the beginning again.
        """

        


if __name__ == '__main__':
    try:
        DummyImagePublisher().run()
    except rospy.ROSInterruptException:
        pass
