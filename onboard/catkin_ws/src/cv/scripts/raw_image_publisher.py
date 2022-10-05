#!/usr/bin/env python3

import rospy
import yaml
import resource_retriever as rr

from custom_msgs.msg import CVObject
from custom_msgs.srv import EnableModel
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from detecto.core import Model

class RawImagePublisher:
    """This class computes and publishes predictions on a image stream."""

    # Load in models and other misc. setup work
    def __init__(self):
        
        rospy.init_node('cv', anonymous=True)
        
        self.bridge = CvBridge()
        self.camera = rospy.get_param('~camera')

        # The topic that the camera publishes its feed to
        self.camera_feed_topic = f'/camera/{self.camera}/image_raw'
        self.publish_path = f'/raw_image/{self.camera}/'
        self.publisher = rospy.Publisher(self.publish_path, Image, queue_size = 10)


    def publish_image(self, img_msg):
        image = self.bridge.imgmsg_to_cv2(img_msg, 'rgb8')
        self.publisher.publish(image)

    def run(self):
        """Initialize node and set up Subscriber to generate and publish predictions at every camera frame received."""
        rospy.Subscriber(self.camera_feed_topic, Image, self.publish_image)

        # Keep node running until shut down
        rospy.spin()

if __name__ == '__main__':
    try:
        RawImagePublisher().run()
    except rospy.ROSInterruptException:
        pass


