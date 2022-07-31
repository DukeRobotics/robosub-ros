#!/usr/bin/env python3

import depthai as dai
import numpy as np
import cv2
import os

# Mock the camera by publishing the same image to a topic
class DummyStreamPublisher:

    # Read in the dummy image and other misc. setup work
    def __init__(self):
        self.pipeline = dai.Pipeline()
        # Dummy still image
        path = os.path.dirname(__file__)
        self.image = cv2.imread(os.path.join(path, 'left384.jpg'),
                                cv2.IMREAD_COLOR)
        # Get path to nn blob file
        self.nnPath = "blobs/yolo_v4_tiny_openvino_2021.3_6shave-2022-7-21_416_416.blob"

    # Publish dummy image to topic every few seconds
    def run(self):

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
        nn.setNumClasses(5)
        nn.setCoordinateSize(4)
        nn.setAnchors(np.array([10, 14, 23, 27, 37, 58, 81, 82, 135, 169, 344, 319]))
        nn.setAnchorMasks({"side26": [0, 1, 2], "side13": [3, 4, 5]})
        nn.setIouThreshold(0.5)

        xIn.out.link(nn.input)
        nn.out.link(xOut.input)

        feedOut = self.pipeline.create(dai.node.XLinkOut)

        feedOut.setStreamName("feed")
        nn.passthrough.link(feedOut.input)

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
            inFeed = qFeed.get()

if __name__ == '__main__':
    DummyStreamPublisher().run()