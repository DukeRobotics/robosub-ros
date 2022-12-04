#!/usr/bin/env python3

import depthai as dai
import numpy as np
import cv2
import os
import depthai_camera_connect
from utils import visualize_detections
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from custom_msgs.msg import CVObject


NN_PATH = os.path.join(os.path.dirname(__file__), '../assets/bloblol.blob')
IMAGE_PATH = os.path.join(os.path.dirname(__file__), '../assets/left384.jpg')


class DepthAISimulateDetection:
    """
    This class is used to test a CV neural network model with a simulated image feed.
    This class takes an image, transfers it from the host (local computer) to the camera,
    and retrieves the output of the neural network.
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

    def detect(self, device, input_image, show_results=False):
        """ Run detection on the input image

        Send the still image through to the input queue (qIn) after converting it to the proper format.
        Then retreive what was fed into the neural network. The input to the neural network should be the same
        as the still image sent into the input queue. Ouput the resulting opencv-formatted image and the
        detections objects.

        Args:
            device (depthai.Device): _description_
            input_image (ndarray): Image to run detection on
            show_results (bool): Whether to display results of detection. Defaults to True.

        Returns:
            dict: Dictionary with keys "frame" and "detections", where frame is the passthrough from the neural network
            and detections is an depthai.ImgDetections object.
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

        if show_results:
            frame = visualize_detections(image_frame, detections)
            cv2.imshow("detections", frame)
            cv2.waitKey(0)

        return {
            "frame": image_frame,
            "detections": detections
        }


class DepthAISimulateDetectionNode(DepthAISimulateDetection):
    """
    This class creates a rosnode that runs detections using the depthai oak camera on a provided
    Image topic or a provided still image.
    """
    def __init__(self):
        super().__init__()
        rospy.init_node('depthai_simulate_detection', anonymous=True)

        self.publishing_topic = rospy.get_param("~publishing_topic")
        self.feed_path = rospy.get_param("~feed_path")

        self.cv_bridge = CvBridge()
        self.publisher = rospy.Publisher(self.publishing_topic, CVObject, queue_size=10)

        self.latest_img = None

    def _load_image(self):
        """ Load a still image from the feed path """
        image_path = os.path.join(os.path.dirname(__file__), self.feed_path)
        image = cv2.imread(image_path, cv2.IMREAD_COLOR)
        self.latest_img = image

    def _feed_is_still_image(self):
        """ Check if the feed_path is to a still image

        Returns:
            bool: Whether the feed_path points to a still image
        """
        try:
            self._load_image()
            return not (self.latest_img is None or self.latest_img.size == 0)
        except Exception:
            return False

    def _update_latest_img(self, img_msg):
        """ Store latest image """
        self.latest_img = self.cv_bridge.imgmsg_to_cv2(img_msg, 'bgr8')

    def _publish_detections(self, detection_results):
        """ Run detection on an image and publish the predictions

        Args:
            detection_results (dict): Output from detect()
        """

        frame = detection_results["frame"]
        detections = detection_results["detections"]

        for detection in detections:

            object_msg = CVObject()

            object_msg.score = detection.confidence
            object_msg.label = str(detection.label)  # TODO: get actual label instead of index

            object_msg.xmin = detection.xmin
            object_msg.ymin = detection.ymin
            object_msg.xmax = detection.xmax
            object_msg.ymax = detection.ymax

            object_msg.height = frame.shape[0]
            object_msg.width = frame.shape[1]

            self.publisher.publish(object_msg)

    def _run_detection_on_image_topic(self, device):
        """ Run and publish detections on the provided topic

        Args:
            device (depthai.Device): Depthai device being used
        """
        rospy.Subscriber(self.feed_path, Image, self._update_latest_img)

        while not rospy.is_shutdown():
            img = self.latest_img
            if img is None:
                continue
            detection_results = self.detect(device, img)
            self._publish_detections(detection_results)

    def run(self):
        """ Run detection on the latest img message """
        with depthai_camera_connect.connect(self.pipeline) as device:
            if self._feed_is_still_image():
                rospy.loginfo(f'Running detection on still image provided: {self.feed_path}')
                self.detect(device, self.latest_img, show_results=True)
                # TODO: exit and kill process?
            else:
                rospy.loginfo(f'Running detection on topic provided: {self.feed_path}')
                self._run_detection_on_image_topic(device)


if __name__ == '__main__':
    try:
        DepthAISimulateDetectionNode().run()
    except rospy.ROSInterruptException:
        pass
