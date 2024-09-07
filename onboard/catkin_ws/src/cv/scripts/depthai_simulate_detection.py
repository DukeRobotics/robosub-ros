#!/usr/bin/env python3

import depthai as dai
import numpy as np
import cv2
import os
import depthai_camera_connect
from utils import DetectionVisualizer
import rospy
import yaml
import resource_retriever as rr
from sensor_msgs.msg import CompressedImage
from custom_msgs.msg import CVObject
from image_tools import ImageTools
import rostopic


class DepthAISimulateDetection:
    """
    This class is used to test a DepthAI neural network (blob file) with a simulated image feed, either a still image
    or an Image topic. This class takes the images, transfers them from the host (local computer) to the camera,
    and retrieves the output of the neural network.
    """
    def __init__(self):
        rospy.init_node('depthai_simulate_detection', anonymous=True)

        self.device = None

        self.feed_path = rospy.get_param("~feed_path")
        self.latest_img = None

        # No feed path is passed in -- throw an expcetion
        if self.feed_path == "":
            rospy.logerr("No feed path variable given")
            rospy.spin()

        self.depthai_model_name = rospy.get_param("~depthai_model_name")
        # No model name is passed in -- throw an expcetion
        if self.depthai_model_name == "":
            rospy.logerr("No model name variable given")
            rospy.spin()

        # Load the model parameters from the yaml file
        with open(rr.get_filename('package://cv/models/depthai_models.yaml', use_protocol=False)) as f:
            models = yaml.safe_load(f)
            self.model = models[self.depthai_model_name]

        # Build NN pipeline using the model weights
        self.nn_path = rr.get_filename(f"package://cv/models/{self.model['weights']}", use_protocol=False)
        self.pipeline = dai.Pipeline()
        self._build_pipeline()

        self.image_tools = ImageTools()

        # Setup detection publishers
        self.publishing_topic = rospy.get_param("~publishing_topic")
        self.detection_publisher = rospy.Publisher(self.publishing_topic, CVObject, queue_size=10)
        self.visualized_detection_publisher = rospy.Publisher(f'{self.publishing_topic}_visualized/compressed',
                                                              CompressedImage,
                                                              queue_size=10)

        # Setup detection visualizer
        self.show_class_name = rospy.get_param("~show_class_name")
        self.show_confidence = rospy.get_param("~show_confidence")
        self.detection_visualizer = DetectionVisualizer(self.model['classes'], self.model['colors'],
                                                        self.show_class_name, self.show_confidence)

    def _build_pipeline(self):
        """
        Build the pipeline. Create an input stream to feed the still image and transfer it to the camera. Link that
        still image stream to the input of the neural network. Create a output link to get the image that was fed
        through the neural network. In the future, we'd like to create an output link to get the predictions from the
        neural network rather than the input to the neural network.
        """

        # Define input stream
        x_in = self.pipeline.create(dai.node.XLinkIn)
        x_in.setStreamName("camIn")

        # Define output stream
        x_out = self.pipeline.create(dai.node.XLinkOut)
        x_out.setStreamName("nn")

        # Define neural net architecture
        nn = self.pipeline.create(dai.node.YoloDetectionNetwork)

        # Neural net / model properties
        nn.setNumInferenceThreads(2)

        nn.setBlobPath(self.nn_path)
        nn.setConfidenceThreshold(self.model['confidence_threshold'])
        nn.input.setBlocking(False)

        nn.setNumClasses(len(self.model['classes']))
        nn.setCoordinateSize(self.model['coordinate_size'])
        nn.setAnchors(np.array(self.model['anchors']))
        nn.setAnchorMasks(self.model['anchor_masks'])
        nn.setIouThreshold(self.model['iou_threshold'])

        # Create a link between the neural net input and the local image stream output
        x_in.out.link(nn.input)
        nn.out.link(x_out.input)

        # Create a new node in the CV/NN pipeline that links to the local image stream
        feed_out = self.pipeline.create(dai.node.XLinkOut)

        # Feed the image stream to the neural net input node
        feed_out.setStreamName("feed")
        nn.passthrough.link(feed_out.input)

    def detect(self, device):
        """ Run detection on the input image

        Send the still image through to the input queue (qIn) after converting it to the proper format.
        Then retreive what was fed into the neural network. The input to the neural network should be the same
        as the still image sent into the input queue. Output the resulting opencv-formatted image and the
        detections objects.

        Args:
            device (depthai.Device): DepthAI device being used

        Returns:
            dict: Dictionary with keys "frame" and "detections", where frame is the passthrough from the neural network
            and detections is an depthai.ImgDetections object.
        """

        passthrough_feed_queue = device.getOutputQueue(
            name="feed", maxSize=4, blocking=False)

        detections_queue = device.getOutputQueue(
            name="nn", maxSize=4, blocking=False)

        passthrough_feed = passthrough_feed_queue.get()

        image_frame = passthrough_feed.getCvFrame()
        detections = detections_queue.get().detections

        return {
            "frame": image_frame,
            "detections": detections
        }

    def _load_image_from_feed_path(self):
        """ Load a still image from the feed path """
        image_path = rr.get_filename(f"package://cv/assets/{self.feed_path}", use_protocol=False)
        image = cv2.imread(image_path, cv2.IMREAD_COLOR)
        return image

    def _feed_is_still_image(self):
        """ Check if the feed_path is to a still image

        Returns:
            bool: Whether the feed_path points to a still image
        """
        try:
            img = self._load_image_from_feed_path()
            return not (img is None or img.size == 0)
        except Exception:
            return False

    def _update_latest_img(self, img_msg):
        """ Send an image to the device for detection

        Args:
            img_msg (sensor_msgs.msg.CompressedImage): Image to send to the device
        """

        # Format a cv2 image to be sent to the device
        def to_planar(arr: np.ndarray, shape: tuple) -> np.ndarray:
            return cv2.resize(arr, shape).transpose(2, 0, 1).flatten()

        latest_img = self.image_tools.convert_to_cv2(img_msg)

        # Input queue will be used to send video frames to the device.
        input_queue = self.device.getInputQueue("camIn")

        # Send a message to the ColorCamera to capture a still image
        img = dai.ImgFrame()
        img.setType(dai.ImgFrame.Type.BGR888p)
        img.setData(to_planar(latest_img, (416, 416)))
        img.setWidth(self.model['input_size'][0])
        img.setHeight(self.model['input_size'][1])
        input_queue.send(img)

    def _publish_detections(self, detection_results):
        """ Run detection on an image and publish the predictions

        Args:
            detection_results (dict): Output from detect()
        """

        frame = detection_results["frame"]
        detections = detection_results["detections"]

        for detection in detections:

            # Construct the message to publish
            object_msg = CVObject()

            object_msg.score = detection.confidence
            object_msg.label = self.model['classes'][detection.label]

            object_msg.xmin = detection.xmin
            object_msg.ymin = detection.ymin
            object_msg.xmax = detection.xmax
            object_msg.ymax = detection.ymax

            object_msg.height = frame.shape[0]
            object_msg.width = frame.shape[1]

            # Publish the message
            self.detection_publisher.publish(object_msg)

    def _publish_visualized_detections(self, detection_results):
        """ Publish the detection results visualized as an Image message

        Args:
            detection_results (dict): Output from detect()
        """
        visualized_detection_results = self.detection_visualizer.visualize_detections(
                                                            detection_results['frame'],
                                                            detection_results['detections'])
        visualized_detection_results_msg = self.image_tools.convert_to_ros_compressed_msg(visualized_detection_results)

        self.visualized_detection_publisher.publish(visualized_detection_results_msg)

    def _run_detection_on_image_topic(self, device):
        """ Run and publish detections on the provided topic

        Args:
            device (depthai.Device): DepthAI device being used
        """
        TopicType, _, _ = rostopic.get_topic_class(self.feed_path)
        rospy.Subscriber(self.feed_path, TopicType, self._update_latest_img)

        while not rospy.is_shutdown():
            detection_results = self.detect(device)
            self._publish_detections(detection_results)
            self._publish_visualized_detections(detection_results)

    def _run_detection_on_single_image(self, device, img):
        """ Run detection on the single image provided

        Args:
            device (depthai.Device): DepthAI device being used
            img (ndarray): Image to run detection on
        """
        self._update_latest_img(img)
        detection_results = self.detect(device)
        visualized_detection_results = self.detection_visualizer.visualize_detections(
                                                                        detection_results['frame'],
                                                                        detection_results['detections'])
        self._save_visualized_detection_results(visualized_detection_results)

    def _save_visualized_detection_results(self, visualized_detection_results):
        """ Save the visualized detection results as a jpg file

        Args:
            visualized_detection_results (ndarray): Image to save
        """
        detection_results_filename = f'{os.path.splitext(self.feed_path)[0]}_detection_results.jpg'
        detection_results_filepath = rr.get_filename(f"package://cv/assets/{detection_results_filename}",
                                                     use_protocol=False)
        cv2.imwrite(detection_results_filepath, visualized_detection_results)

    def run(self):
        """ Run detection on the latest img message """
        with depthai_camera_connect.connect(self.pipeline) as device:
            self.device = device

            if self._feed_is_still_image():
                rospy.loginfo(f'Running detection on still image provided: {self.feed_path}')
                img = self._load_image_from_feed_path()
                self._run_detection_on_single_image(device, img)
            else:
                rospy.loginfo(f'Running detection on topic provided: {self.feed_path}')
                self._run_detection_on_image_topic(device)


if __name__ == '__main__':
    try:
        DepthAISimulateDetection().run()
    except rospy.ROSInterruptException:
        pass
