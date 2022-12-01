#!/usr/bin/env python3

import depthai as dai
import numpy as np
import cv2
import os
import depthai_camera_connect
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from custom_msgs.msg import CVObject


IMAGE_STREAM_TOPIC = '/cv/camera/raw_image'
DETECTION_RESULTS_TOPIC = '/cv/sim_cam/detections'


class DepthAISimulateSpatialDetectionROS(DepthAISimulateSpatialDetection):
    def __init__(self):
        super().__init__()
        rospy.init_node('depthai_simulated_spatial_detection', anonymous=True)

        self.cv_bridge = CvBridge()
        self.publisher = rospy.Publisher(DETECTION_RESULTS_TOPIC, Image, queue_size=10)
        rospy.Subscriber(IMAGE_STREAM_TOPIC, Image, self._add_image_message_to_queue)

        self.latest_img_msg = None

    def _add_image_message_to_queue(self, img_msg):
        """ Store latest image """
        self.latest_img_msg = img_msg

    def run():
        """ Run detection on the latest img message """
        with depthai_camera_connect.connect(self.pipeline) as device:
            while not rospy.is_shutdown():
                img_msg = self.latest_img_msg
                image = self.bridge.imgmsg_to_cv2(img_msg, 'rgb8')

                out = self.detect(device, image)

                frame = out["frame"]
                detections = out["detections"]

                for detection in detections:

                    object_msg = CVObject()

                    object_msg.label = "object"
                    object_msg.score = detection.confidence

                    object_msg.xmin = detection.xmin
                    object_msg.ymin = detection.ymin
                    object_msg.xmax = detection.xmax
                    object_msg.ymax = detection.ymax

                    object_msg.height = frame.shape[0]
                    object_msg.width = frame.shape[1]
                    
                    self.publisher.publish(object_msg)


if __name__ == '__main__':
    try:
        DepthAISimulateSpatialDetectionROS().run()
    except rospy.ROSInterruptException:
        pass
