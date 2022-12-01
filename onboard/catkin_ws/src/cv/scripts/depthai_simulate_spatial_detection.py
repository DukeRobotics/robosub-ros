#!/usr/bin/env python3

import depthai as dai
import numpy as np
import cv2
import os
import depthai_camera_connect

path = os.path.dirname(__file__)
NN_PATH = os.path.join(path, '../assets/bloblol.blob')
IMAGE_PATH = os.path.join(path, '../assets/left384.jpg')


class DepthAISimulateSpatialDetection:
    """
    This class is used to test a CV neural network model locally with a simulated image feed.
    This class takes a still image and transfers it from the host (local computer) to the camera. The
    image feed is then ran through the provided neural network.

    This file can be used for local testing outside of the docker container since
    it does not require ROS.
    """

    # Read in the dummy image and other misc. setup work
    def __init__(self):
        self.nnPath = NN_PATH
        self.pipeline = dai.Pipeline()
        self._build_pipeline()

        # Dummy still image
        self.image = cv2.imread(IMAGE_PATH, cv2.IMREAD_COLOR)

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
        xOut.setStreamName("nn")

        # Define neural net architecture
        nn = self.pipeline.create(dai.node.YoloDetectionNetwork)

        # Neural net / model properties
        nn.setConfidenceThreshold(0.5)
        nn.setBlobPath(self.nnPath)
        nn.setNumInferenceThreads(2)
        nn.input.setBlocking(False)
        nn.setNumClasses(5)
        nn.setCoordinateSize(4)
        nn.setAnchors(
            np.array([10, 14, 23, 27, 37, 58, 81, 82, 135, 169, 344, 319]))
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

    def detect_single_image(self, img, show=True):
        """ Run detection on a single provided image """
        with depthai_camera_connect.connect(self.pipeline) as device:
            out = self.detect(device, img)
            if show:
                display_frame("detections", out["frame"], out["detections"])
            return out

    def detect(self, device, input_image):
        """
        Send the still image through to the input queue (qIn) after converting it to the proper format.
        Then retreive what was fed into the neural network. The input to the neural network should be the same
        as the still image sent into the input queue. Ouput the resulting opencv-formatted image and the
        detections objects.
        """

        # Upload the pipeline to the device
        def to_planar(arr: np.ndarray, shape: tuple) -> np.ndarray:
            return cv2.resize(arr, shape).transpose(2, 0, 1).flatten()

        # Input queue will be used to send video frames to the device.
        input_queue = device.getInputQueue("camIn")

        passthrough_feed_queue = device.getOutputQueue(
            name="feed", maxSize=4, blocking=False)

        detections_queue = device.getOutputQueue(
            name="nn", maxSize=4, blocking=False)

        # Send a message to the ColorCamera to capture a still image
        img = dai.ImgFrame()
        img.setType(dai.ImgFrame.Type.BGR888p)
        img.setData(to_planar(input_image, (416, 416)))
        img.setWidth(416)
        img.setHeight(416)
        input_queue.send(img)

        passthrough_feed = passthrough_feed_queue.get()

        image_frame = passthrough_feed.getCvFrame()
        detections = detections_queue.get().detections

        return {
            "frame": image_frame,
            "detections": detections
        }


def frame_norm(frame, bbox):
    """ Normalize bbox locations between frame width/height """
    norm_vals = np.full(len(bbox), frame.shape[0])
    norm_vals[::2] = frame.shape[1]
    return (np.clip(np.array(bbox), 0, 1) * norm_vals).astype(int)


def display_frame(name, frame, detections):
    """ Display frame and nn detections """
    color = (255, 0, 0)
    for detection in detections:
        bbox = frame_norm(frame, (detection.xmin, detection.ymin, detection.xmax, detection.ymax))
        cv2.putText(frame, f"{int(detection.confidence * 100)}%", (bbox[0] + 10, bbox[1] + 40), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
        cv2.rectangle(frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), color, 2)
    cv2.imshow(name, frame)
    cv2.waitKey(-1)


if __name__ == '__main__':
    d = DepthAISimulateSpatialDetection()
    out = d.detect_single_image(d.image)
