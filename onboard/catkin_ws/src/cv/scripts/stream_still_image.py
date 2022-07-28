#!/usr/bin/env python3

import rospy
import depthai as dai
import numpy as np
import cv2
import os
from custom_msgs.srv import EnableModel
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Mock the camera by publishing the same image to a topic
class DummyStreamPublisher:

    CAMERA = 'front'
    STREAM_TOPIC = f'/camera/{CAMERA}/stream_still_image'

    # Read in the dummy image and other misc. setup work
    def __init__(self):
        rospy.init_node("stream_still_image")
        self.stream_publisher = rospy.Publisher(self.STREAM_TOPIC, Image,
                                               queue_size=10)

        self.bridge = CvBridge()
        self.pipeline = dai.Pipeline()

        # Dummy still image
        path = os.path.dirname(__file__)
        self.image = cv2.imread(os.path.join(path, '../assets/left384.jpg'),
                           cv2.IMREAD_COLOR)

        # Get path to nn blob file
        self.nnPath = ""


    # Publish dummy image to topic every few seconds
    def run(self):

        loop_rate = rospy.Rate(1)

        # Point xIn to still image
        xIn = self.pipeline.create(dai.node.XLinkIn)
        xOut = self.pipeline.create(dai.node.XLinkOut)
        xIn.setStreamName("camIn")
        xOut.setStreamName("camOut")

        # Define neural net architecture
        nn = self.pipeline.create(dai.node.YoloDetectionNetwork)

        # Neural net properties
        nn.setConfidenceThreshold(0.5)
        nn.setBlobPath(self.nnPath)
        nn.setNumInferenceThreads(2)
        nn.input.setBlocking(False)
        
        xIn.out.link(nn.input)
        nn.out.link(xOut.input)

        # Upload the pipeline to the device
        with dai.Device(self.pipeline) as device:

            def to_planar(arr: np.ndarray, shape: tuple) -> np.ndarray:
                return cv2.resize(arr, shape).transpose(2, 0, 1).flatten()

            # Input queue will be used to send video frames to the device.
            qIn = device.getInputQueue("camIn")
            # Output queue will be used to get nn data from the video frames.
            qOut = device.getOutputQueue(name="nn", maxSize=4, blocking=False)

            # Send a message to the ColorCamera to capture a still image
            img = dai.ImgFrame()
            img.setData(to_planar(self.image, (416, 416)))
            qIn.send(img)

            inDet = qOut.get()
            loop_rate.sleep()