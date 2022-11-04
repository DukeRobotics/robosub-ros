#!/usr/bin/env python3

import depthai as dai
import numpy as np
import cv2
import os
import rospy

path = os.path.dirname(__file__)
NN_PATH = os.path.join(path, '../assets/bloblol.blob')
IMAGE_RELATIVE_PATH = os.path.join(path, '../assets/left384.jpg')


class DepthAIMockImageStream:
    """
    THIS FILE IS INCOMPLETE. It can be used for local testing outside of the docker container since
    it does not require ROS.

    This class is used to test a CV neural network model locally with a simulated image feed.
    This class takes a still image and transfers it from the host (local computer) to the camera. The
    image feed is then ran through the provided neural network.
    """

    CAMERA = 'left'
    IMAGE_TOPIC = f'/camera/{CAMERA}/image_raw'
    # Read in the dummy image and other misc. setup work
    def __init__(self):
        self.nnPath = NN_PATH
        self.pipeline = dai.Pipeline()
        self._build_pipeline()

        # Dummy still image
        path = os.path.dirname(__file__)
        self.image = cv2.imread(os.path.join(path, IMAGE_RELATIVE_PATH), cv2.IMREAD_COLOR)
        self.subscribe = rospy.Subscriber(self.IMAGE_TOPIC, queue_size=10)
        
        # Get path to nn blob file

    def _build_pipeline(self):
        """
        Build the pipeline. Create an input stream to feed the still image and transfer it to the camera. Link that
        still image stream to the input of the neural network. Create a output link to get the image that was fed
        through the neural network. In the future, we'd like to create an output link to get the predictions from the
        neural network rather than the input to the neural network.
        """

        # Point xIn to still image
        xIn = self.pipeline.create(dai.node.XLinkIn)

        xOut = self.pipeline.create(dai.node.XLinkOut)
        xIn.setStreamName("camIn")
        xOut.setStreamName("camOut")

        # Define neural net architecture
        nn = self.pipeline.create(dai.node.YoloDetectionNetwork)

        # Neural net / model properties
        nn.setConfidenceThreshold(0.5)
        nn.setBlobPath(self.nnPath)
        nn.setNumInferenceThreads(2)
        nn.input.setBlocking(False)
        nn.setNumClasses(5)
        nn.setCoordinateSize(4)
        nn.setAnchors(np.array([10, 14, 23, 27, 37, 58, 81, 82, 135, 169, 344, 319]))
        nn.setAnchorMasks({"side26": [0, 1, 2], "side13": [3, 4, 5]})
        nn.setIouThreshold(0.5)

        # Create a link between the neural net input and the local image stream output
        xIn.out.link(nn.input)
        nn.out.link(xOut.input)

        # Create a new node in the CV/NN pipeline that links to the local image stream
        feedOut = self.pipeline.create(dai.node.XLinkOut)

        # Feed the image stream to the neural net input node
        feedOut.setStreamName("feed")
        nn.passthrough.link(feedOut.input)

    # Publish dummy image to topic every few seconds
    def run(self):
        """
        Send the still image through to the input queue (qIn) after converting it to the proper format.
        Then retreive what was fed into the neural network. The input to the neural network should be the same
        as the still image sent into the input queue.
        """

        # Upload the pipeline to the device
        with dai.Device(self.pipeline) as device:

            def to_planar(arr: np.ndarray, shape: tuple) -> np.ndarray:
                return cv2.resize(arr, shape).transpose(2, 0, 1).flatten()

            # Input queue will be used to send video frames to the device.
            qIn = device.getInputQueue("camIn")

            # Output queue will be used to get nn data from the video frames.
            qFeed = device.getOutputQueue(name="feed", maxSize=4, blocking=False)
            #qOut = device.getOutputQueue(name="nn", maxSize=4, blocking=False)

            # Send a message to the ColorCamera to capture a still image
            img = dai.ImgFrame()
            img.setType(dai.ImgFrame.Type.BGR888p)
            img.setData(to_planar(self.image, (416, 416)))

            img.setWidth(416)
            img.setHeight(416)

            qIn.send(img)
            # inFeed = qFeed.get()


if __name__ == '__main__':
    DepthAIMockImageStream().run()
