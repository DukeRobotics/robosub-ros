#!/usr/bin/env python3

import rospy
import depthai as dai
import cv2
import os
from custom_msgs.srv import EnableModel
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


# Stream images from camera to host
class StreamPublisher:

    CAMERA = 'front'
    STREAM_TOPIC = f'/camera/{CAMERA}/stream_raw'

    # Set up publisher and camera node pipeline
    def __init__(self):
        rospy.init_node('stream_images')
        self.stream_publisher = rospy.Publisher(self.STREAM_TOPIC, Image,
                                               queue_size=10)

        self.bridge = CvBridge()
        self.pipeline = dai.Pipeline()


    # Publish newest image off queue to topic every few seconds
    def run(self):
        loop_rate = rospy.Rate(1)

        # Define source and output
        camRgb = self.pipeline.create(dai.node.ColorCamera)
        xoutRgb = self.pipeline.create(dai.node.XLinkOut)

        xoutRgb.setStreamName("rgb")

        # Properties
        camRgb.setPreviewSize(300, 300)
        camRgb.setInterleaved(False)
        camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)

        # Linking
        camRgb.preview.link(xoutRgb.input)

        # Upload the pipeline to the device
        with dai.Device(self.pipeline) as device:

            # Output queue, to receive message on the host from the device (you can send the message on the device with XLinkOut)
            rgbQueue = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
            
            while not rospy.is_shutdown():

                # Get a message that came from the queue
                raw_img = rgbQueue.get()
                img = raw_img.getCvFrame()

                # Publish the image
                image_msg = self.bridge.cv2_to_imgmsg(img, 'bgr8')
                self.stream_publisher.publish(image_msg)

            loop_rate.sleep()

if __name__ == '__main__':
    try:
        StreamPublisher().run()
    except rospy.ROSInterruptException:
        pass
