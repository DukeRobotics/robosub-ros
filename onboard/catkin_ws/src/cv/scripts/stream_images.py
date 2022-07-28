#!/usr/bin/env python3

import rospy
import depthai as dai
import cv2
import os
from custom_msgs.srv import EnableModel
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


# Mock the camera by publishing the same image to a topic
class StreamPublisher:

    NODE_NAME = 'test_stream'
    CAMERA = 'front'
    STREAM_TOPIC = f'/camera/{CAMERA}/stream_raw'

    # Read in the dummy image and other misc. setup work
    def __init__(self):
        self.stream_publisher = rospy.Publisher(self.STREAM_TOPIC, Image,
                                               queue_size=10)

        self.bridge = CvBridge()
        self.pipeline = dai.Pipeline()


    # Publish dummy image to topic every few seconds
    def run(self):

        rospy.init_node(self.NODE_NAME)
        loop_rate = rospy.Rate(1)

        count = 0
        # Upload the pipeline to the device
        with dai.Device(self.pipeline) as device:

            # Output queue, to receive message on the host from the device (you can send the message on the device with XLinkOut)
            output_q = device.getOutputQueue("output_name", maxSize=4, blocking=False)
            
            while not rospy.is_shutdown():

                # Get a message that came from the queue
                raw_img = output_q.tryGet()
                img = raw_img.getCvFrame()

                # Publish the image
                image_msg = self.bridge.cv2_to_imgmsg(img, 'bgr8')
                self.stream_publisher.publish(image_msg)

            # count += 1
            loop_rate.sleep()

if __name__ == '__main__':
    StreamPublisher().run()
