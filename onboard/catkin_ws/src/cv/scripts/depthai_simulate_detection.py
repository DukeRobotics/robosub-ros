#!/usr/bin/env python3

import depthai as dai
import numpy as np
import cv2
import os
import depthai_camera_connect
import rospy
import yaml
import resource_retriever as rr
import rostopic

from utils import DetectionVisualizer
from image_tools import ImageTools
from sensor_msgs.msg import CompressedImage
from custom_msgs.msg import CVObject
from custom_msgs.srv import SetDepthAIModel
from std_srvs.srv import SetBool


DEPTHAI_SCRIPT_NODES_PATH = 'package://cv/scripts/depthai_script_nodes/'


class DepthAISimulateDetection:
    """
    This class is used to test a DepthAI neural network (blob file) with a simulated image feed, either a still image
    or an Image topic. This class takes the images, transfers them from the host (local computer) to the camera,
    and retrieves the output of the neural network.
    """
    def __init__(self):
        rospy.init_node('depthai_simulate_detection', anonymous=True)

        self.device = None
        self.camera = "front"

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
            self.models = yaml.safe_load(f)
            self.current_model = self.models[self.depthai_model_name]

        # Build NN pipeline using the model weights
        # self.nn_path = rr.get_filename(f"package://cv/models/{self.model['weights']}", use_protocol=False)
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
        self.detection_visualizer = DetectionVisualizer(self.current_model['classes'], self.current_model['colors'],
                                                        self.show_class_name, self.show_confidence)

        # Allow service for toggling of models
        rospy.Service(f'simulate_set_model_{self.camera}', SetDepthAIModel, self.set_model)
        rospy.Service(f'simulate_enable_model_{self.camera}', SetBool, self.enable_model)

        self.detections_enabled = True

    def _build_pipeline(self):
        """
        Build the pipeline. Create an input stream to feed the still image and transfer it to the camera. Link that
        still image stream to the input of the neural network. Create a output link to get the image that was fed
        through the neural network. In the future, we'd like to create an output link to get the predictions from the
        neural network rather than the input to the neural network.
        """

        # Define input streams
        xin_cam = self.pipeline.create(dai.node.XLinkIn)
        xin_cam.setStreamName("camIn")

        xin_model = self.pipeline.create(dai.node.XLinkIn)
        xin_model.setStreamName("model")

        # Define output stream
        x_out = self.pipeline.create(dai.node.XLinkOut)
        x_out.setStreamName("nn")

        # Create a new node in the CV/NN pipeline that links to the local image stream
        feed_out = self.pipeline.create(dai.node.XLinkOut)
        feed_out.setStreamName("feed")

        # Script nodes to handle switching models on the fly
        switch_model = self.pipeline.create(dai.node.Script)
        switch_model_path = DEPTHAI_SCRIPT_NODES_PATH + 'simulate_switch_model.py'
        with open(rr.get_filename(switch_model_path, use_protocol=False)) as f:
            switch_model.setScript(f.read())

        # Neural network architecture nodes
        for model_name in self.models:
            model = self.models[model_name]

            blob_path = rr.get_filename(f"package://cv/models/{model['weights']}", use_protocol=False)

            # Set up a new network for each model
            nn = self.pipeline.create(dai.node.YoloDetectionNetwork)

            nn.setBlobPath(blob_path)
            nn.setConfidenceThreshold(model['confidence_threshold'])
            nn.setNumInferenceThreads(2)
            nn.input.setBlocking(False)

            # Yolo specific parameters
            nn.setNumClasses(len(model['classes']))
            nn.setCoordinateSize(model['coordinate_size'])
            nn.setAnchors(np.array(model['anchors']))
            nn.setAnchorMasks(model['anchor_masks'])
            nn.setIouThreshold(model['iou_threshold'])

            # Linking switch node outputs to spatial detection network inputs
            switch_model.outputs[f"{model['id']}_input"].link(nn.input)

            nn.passthrough.link(switch_model.inputs[f"{model['id']}_passthrough"])
            nn.out.link(switch_model.inputs[f"{model['id']}_out"])

        # Linking
        xin_model.out.link(switch_model.inputs["model"])

        xin_cam.out.link(switch_model.inputs["input"])

        # Create a link between the neural net input and the local image stream output
        switch_model.outputs["out"].link(x_out.input)
        switch_model.outputs["passthrough"].link(feed_out.input)

    def detect(self, device):
        """
        Run detection on the input image.

        Send the still image through to the input queue (qIn) after converting it to the proper format.
        Then retrieve what was fed into the neural network. The input to the neural network should be the same
        as the still image sent into the input queue. Output the resulting opencv-formatted image and the
        detections objects.

        Args:
            device (depthai.Device): DepthAI device being used

        Returns:
            dict: Dictionary with keys "frame" and "detections", where frame is the passthrough from the neural network
            and detections is an depthai.ImgDetections object.
        """

        passthrough_feed_queue = device.getOutputQueue(name="feed", maxSize=4, blocking=False)

        detections_queue = device.getOutputQueue(name="nn", maxSize=4, blocking=False)

        passthrough_feed = passthrough_feed_queue.tryGet()
        detections = detections_queue.tryGet()

        return {
            "frame": passthrough_feed.getCvFrame() if passthrough_feed else None,
            "detections": detections.detections if detections else None
        }

    def _load_image_from_feed_path(self):
        """
        Load a still image from the feed path.
        """
        image_path = rr.get_filename(f"package://cv/assets/{self.feed_path}", use_protocol=False)
        image = cv2.imread(image_path, cv2.IMREAD_COLOR)
        return image

    def _feed_is_still_image(self):
        """
        Check if the feed_path is to a still image.

        Returns:
            bool: Whether the feed_path points to a still image
        """
        try:
            img = self._load_image_from_feed_path()
            return not (img is None or img.size == 0)
        except Exception:
            return False

    def _update_latest_img(self, img_msg):
        """
        Send an image to the device for detection.

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
        img.setWidth(self.current_model['input_size'][0])
        img.setHeight(self.current_model['input_size'][1])
        input_queue.send(img)

    def _publish_detections(self, detection_results):
        """
        Run detection on an image and publish the predictions.

        Args:
            detection_results (dict): Output from detect()
        """
        frame = detection_results["frame"]
        detections = detection_results["detections"]

        for detection in detections:

            # Construct the message to publish
            object_msg = CVObject()

            object_msg.score = detection.confidence
            object_msg.label = self.current_model['classes'][detection.label]

            object_msg.xmin = detection.xmin
            object_msg.ymin = detection.ymin
            object_msg.xmax = detection.xmax
            object_msg.ymax = detection.ymax

            object_msg.height = frame.shape[0]
            object_msg.width = frame.shape[1]

            # Publish the message
            self.detection_publisher.publish(object_msg)

    def _publish_visualized_detections(self, detection_results):
        """
        Publish the detection results visualized as an Image message.

        Args:
            detection_results (dict): Output from detect()
        """
        visualized_detection_results = self.detection_visualizer.visualize_detections(
                                                            detection_results['frame'],
                                                            detection_results['detections'])
        visualized_detection_results_msg = self.image_tools.convert_to_ros_compressed_msg(visualized_detection_results)

        self.visualized_detection_publisher.publish(visualized_detection_results_msg)

    def _run_detection_on_image_topic(self, device):
        """
        Run and publish detections on the provided topic.

        Args:
            device (depthai.Device): DepthAI device being used
        """
        TopicType, _, _ = rostopic.get_topic_class(self.feed_path)
        rospy.Subscriber(self.feed_path, TopicType, self._update_latest_img)

        while not rospy.is_shutdown() and self.detections_enabled:
            detection_results = self.detect(device)

            if detection_results["detections"]:
                self._publish_detections(detection_results)

                if detection_results["frame"]:
                    self._publish_visualized_detections(detection_results)

    # This wouldn't work due to the non-blocking nature of tryGet which is needed for this pipeline
    def _run_detection_on_single_image(self, device, img):
        """
        Run detection on the single image provided.

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
        """
        Save the visualized detection results as a jpg file.

        Args:
            visualized_detection_results (ndarray): Image to save
        """
        detection_results_filename = f'{os.path.splitext(self.feed_path)[0]}_detection_results.jpg'
        detection_results_filepath = rr.get_filename(f"package://cv/assets/{detection_results_filename}",
                                                     use_protocol=False)
        cv2.imwrite(detection_results_filepath, visualized_detection_results)

    def set_model(self, req):
        new_model_name = req.model_name

        if new_model_name == self.depthai_model_name:
            return {"success": True, "message": f"Model {new_model_name} is already running on {self.camera} camera."}

        if new_model_name not in self.models:
            return {"success": False, "message": f"FAILURE: {new_model_name} is not a valid model name."}

        else:
            self.detections_enabled = False

            self.depthai_model_name = new_model_name
            self.current_model = self.models[new_model_name]

            self.detection_visualizer = DetectionVisualizer(self.current_model['classes'], self.current_model['colors'],
                                                            self.show_class_name, self.show_confidence)

            # Send model name to the pipeline
            input_model_msg = dai.Buffer()
            input_model_msg.setData(self.current_model['id'])
            self.device.getInputQueue("model").send(input_model_msg)

            self.detections_enabled = True

            return {"success": True, "message": f"Sucessfully set new model {new_model_name} for {self.camera} camera."}

    def enable_model(self, req):
        """
        Service for toggling specific models on and off.

        :param req: The request from another node or command line to enable the model.
        """
        input_model_msg = dai.Buffer()
        if req.data:
            input_model_msg.setData(self.current_model['id'])
            message = "Successfully enabled model."
        else:
            input_model_msg.setData(0)
            message = "Successfully disabled model."
        self.device.getInputQueue("model").send(input_model_msg)

        return {"success": True, "message": message}

    def run(self):
        """
        Run detection on the latest img message.
        """
        with depthai_camera_connect.connect(self.pipeline, self.camera) as device:
            self.device = device

            # Send model name to the pipeline
            input_model_msg = dai.Buffer()
            input_model_msg.setData(self.current_model['id'])
            self.device.getInputQueue("model").send(input_model_msg)

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
