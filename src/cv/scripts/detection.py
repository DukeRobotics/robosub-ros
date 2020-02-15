#!/usr/bin/env python

import rospy
import os
from cv.msg import Object
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from detecto.core import Model


class Detector:

    NODE_NAME = 'cv'
    GATE_TOPIC = '/cv/buoy'
    IMAGE_TOPIC = '/test_images/image'

    BUOY_CLASSES = ['alien', 'bat', 'witch', 'brick', '_start_gate', '_start_tick', '_']

    # Load in models and other misc. setup work
    def __init__(self):
        self.buoy_publisher = rospy.Publisher(self.GATE_TOPIC, Object, queue_size=10)
        self.bridge = CvBridge()

        path = os.path.dirname(__file__)
        buoy_file = os.path.join(path, '../models/buoy.pth')
        self.buoy_model = Model.load(buoy_file, self.BUOY_CLASSES)

    # Camera subscriber callback; publishes predictions for each frame
    def detect(self, img_msg):
        image = self.bridge.imgmsg_to_cv2(img_msg, 'rgb8')

        preds = self.buoy_model.predict_top(image)

        for label, box, score in zip(*preds):
            object_msg = Object()

            object_msg.label = label
            object_msg.score = score

            object_msg.xmin = box[0].item()
            object_msg.ymin = box[1].item()
            object_msg.xmax = box[2].item()
            object_msg.ymax = box[3].item()

            self.buoy_publisher.publish(object_msg)

    # Initialize node and set up Subscriber to generate and
    # publish predictions at every camera frame
    def run(self):
        rospy.init_node(self.NODE_NAME)
        rospy.Subscriber(self.IMAGE_TOPIC, Image, self.detect)

        while not rospy.is_shutdown():
            pass  # There's gotta be a better way to do this


if __name__ == '__main__':
    Detector().run()
