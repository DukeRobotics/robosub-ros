#!/usr/bin/env python3

import rospy
import resource_retriever as rr
import yaml
import math
import depthai_camera_connect
import depthai as dai
import numpy as np
from utils import DetectionVisualizer, calculate_relative_pose
from image_tools import ImageTools
import cv2

from custom_msgs.msg import CVObject, SonarSweepRequest, SonarSweepResponse
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

CAMERA_CONFIG_PATH = 'package://cv/configs/usb_cameras.yaml'
with open(rr.get_filename(CAMERA_CONFIG_PATH, use_protocol=False)) as f:
    cameras = yaml.safe_load(f)
CAMERA = cameras['front']  # TODO: use ROS params to select camera

MM_IN_METER = 1000
DEPTHAI_OBJECT_DETECTION_MODELS_FILEPATH = 'package://cv/models/depthai_models.yaml'
HORIZONTAL_FOV = 95
SONAR_DEPTH = 10
SONAR_RANGE = 1.75
SONAR_REQUESTS_PATH = 'sonar/request'
SONAR_RESPONSES_PATH = 'sonar/cv/response'
TASK_PLANNING_REQUESTS_PATH = "controls/desired_feature"

GATE_IMAGE_WIDTH = 0.2452  # Width of gate images in meters
GATE_IMAGE_HEIGHT = 0.2921  # Height of gate images in meters

# Camera constants
FOCAL_LENGTH = CAMERA['focal_length']  # Focal length of camera in mm
SENSOR_SIZE = (CAMERA['sensor_size']['width'], CAMERA['sensor_size']['height'])  # Sensor size in mm


# Compute detections on live camera feed and publish spatial coordinates for detected objects
class DepthAISpatialDetector:
    def __init__(self):
        """
        Initializes the ROS node. Loads the yaml file at cv/models/depthai_models.yaml
        """
        rospy.init_node('depthai_spatial_detection', anonymous=True)
        self.feed_path = rospy.get_param("~feed_path")
        self.running_model = rospy.get_param("~model")
        self.rgb_raw = rospy.get_param("~rgb_raw")
        self.rgb_detections = rospy.get_param("~rgb_detections")
        self.queue_depth = rospy.get_param("~depth")  # Whether to output depth map
        self.sync_nn = rospy.get_param("~sync_nn")
        self.using_sonar = rospy.get_param("~using_sonar")
        self.show_class_name = rospy.get_param("~show_class_name")
        self.show_confidence = rospy.get_param("~show_confidence")
        self.device = None

        with open(rr.get_filename(DEPTHAI_OBJECT_DETECTION_MODELS_FILEPATH,
                                  use_protocol=False)) as f:
            self.models = yaml.safe_load(f)

        self.camera = 'front'
        self.pipeline = None
        self.publishers = {}  # Keys are the class names of a given model
        self.output_queues = {}  # Keys are "rgb", "depth", and "detections"
        self.connected = False
        self.current_model_name = None
        self.classes = None
        self.camera_pixel_width = None
        self.camera_pixel_height = None
        self.detection_feed_publisher = None
        self.rgb_preview_publisher = None
        self.detection_visualizer = None

        self.image_tools = ImageTools()

        self.sonar_response = (0, 0)
        self.in_sonar_range = True

        # By default the first task is going through the gate
        self.current_priority = "buoy_abydos_serpenscaput"

        # Initialize publishers and subscribers for sonar/task planning
        self.sonar_requests_publisher = rospy.Publisher(
            SONAR_REQUESTS_PATH, SonarSweepRequest, queue_size=10)
        self.sonar_response_subscriber = rospy.Subscriber(
            SONAR_RESPONSES_PATH, SonarSweepResponse, self.update_sonar)
        self.desired_detection_feature = rospy.Subscriber(
            TASK_PLANNING_REQUESTS_PATH, String, self.update_priority)

        rospy.Subscriber(self.feed_path, CompressedImage, self._update_latest_img, queue_size=1)

    def _update_latest_img(self, img_msg):
        """ Send an image to the device for detection

        Args:
            img_msg (sensor_msgs.msg.CompressedImage): Image to send to the device
        """
        model = self.models[self.current_model_name]

        # Format a cv2 image to be sent to the device
        def to_planar(arr: np.ndarray, shape: tuple) -> np.ndarray:
            return cv2.resize(arr, shape).transpose(2, 0, 1).flatten()

        latest_img = self.image_tools.convert_to_cv2(img_msg)

        # Input queue will be used to send video frames to the device.
        if self.device:
            input_queue = self.device.getInputQueue("nn_input")

            # Send a message to the ColorCamera to capture a still image
            img = dai.ImgFrame()
            img.setType(dai.ImgFrame.Type.BGR888p)

            img.setData(to_planar(latest_img, (416, 416)))

            img.setWidth(model['input_size'][0])
            img.setHeight(model['input_size'][1])

            input_queue.send(img)

    def build_pipeline(self, nn_blob_path, sync_nn):
        """
        Get the DepthAI Pipeline for 3D object localization. Inspiration taken from
        https://docs.luxonis.com/projects/api/en/latest/samples/SpatialDetection/spatial_tiny_yolo/.
        To understand the DepthAI pipeline structure, please see https://docs.luxonis.com/projects/api/en/latest/.
        This pipeline computes the depth map using the two mono cameras. This depth map and the RGB feed are fed into
        the YoloSpatialDetection Node, which detects objects and computes the average depth within the bounding box
        for any detected object. The object detection model for this node is loaded from the nnBlobPath. For info
        about the YoloSpatialDetection Node, see
        https://docs.luxonis.com/projects/api/en/latest/components/nodes/yolo_spatial_detection_network/.
        The output queues available from this pipeline are:
            - "rgb": contains the 400x400 RGB preview of the camera feed.
            - "detections": contains SpatialImgDetections messages (https://docs.luxonis.com/projects/api/en/latest/
            components/messages/spatial_img_detections/#spatialimgdetections), which includes bounding boxes for
            detections as well as XYZ coordinates of the detected objects.
            - "depth": contains ImgFrame messages with UINT16 values representing the depth in millimeters by default.
                See the property depth in https://docs.luxonis.com/projects/api/en/latest/components/nodes/stereo_depth/

        :param nn_blob_path: Path to blob file used for object detection.
        :param sync_nn: If True, sync the RGB output feed with the detection from the neural network. Needed if the RGB
        feed output will be used and needs to be synced with the object detections.
        :return: depthai.Pipeline object to compute
        """
        model = self.models[self.current_model_name]

        pipeline = dai.Pipeline()

        # Define sources and outputs
        spatial_detection_network = pipeline.create(dai.node.YoloDetectionNetwork)
        feed_out = pipeline.create(dai.node.XLinkOut)

        xout_nn = pipeline.create(dai.node.XLinkOut)
        xout_nn.setStreamName("detections")

        xin_nn_input = pipeline.create(dai.node.XLinkIn)
        xin_nn_input.setStreamName("nn_input")
        xin_nn_input.setNumFrames(2)
        xin_nn_input.setMaxDataSize(416*416*3)

        # General spatial detection network parameters
        spatial_detection_network.setBlobPath(nn_blob_path)
        spatial_detection_network.setConfidenceThreshold(model['confidence_threshold'])
        spatial_detection_network.input.setBlocking(False)

        # Yolo specific parameters
        spatial_detection_network.setNumClasses(len(model['classes']))
        spatial_detection_network.setCoordinateSize(model['coordinate_size'])
        spatial_detection_network.setAnchors(np.array(model['anchors']))
        spatial_detection_network.setAnchorMasks(model['anchor_masks'])
        spatial_detection_network.setIouThreshold(model['iou_threshold'])

        # Linking
        xin_nn_input.out.link(spatial_detection_network.input)

        spatial_detection_network.out.link(xout_nn.input)

        # Feed the image stream to the neural net input node
        feed_out.setStreamName("feed")
        spatial_detection_network.passthrough.link(feed_out.input)

        return pipeline

    def init_model(self, model_name):
        """
        Creates and assigns the pipeline and sets the current model name.

        :param model_name: Name of the model. The model name should match a key in cv/models/depthai_models.yaml.
        For example, if depthai_models.yaml is:

        gate:
            classes: ['gate', 'gate_side', 'gate_tick', 'gate_top', 'start_gate']
            topic: cv/
            weights: yolo_v4_tiny_openvino_2021.3_6shave-2022-7-21_416_416.blob
            input_size: [416, 416]
            coordinate_size: 4
            anchors: [10, 14, 23, 27, 37, 58, 81, 82, 135, 169, 344, 319]
            anchor_masks: {"side26": [0, 1, 2], "side13": [3, 4, 5]}

        Then, a possible model name is "gate".
        """
        # If the model is already set, don't reinitialize
        if model_name == self.current_model_name:
            return

        self.current_model_name = model_name

        model = self.models[model_name]

        self.classes = model['classes']

        self.camera_pixel_width, self.camera_pixel_height = model['input_size']

        self.colors = model['colors']

        blob_path = rr.get_filename(f"package://cv/models/{model['weights']}",
                                    use_protocol=False)
        self.pipeline = self.build_pipeline(blob_path, self.sync_nn)

    def init_publishers(self, model_name):
        """
        Initialize the publishers for the node. A publisher is created for each class that the model predicts.
        The publishers are created in format: "cv/camera/model_name".
        :param model_name: Name of the model that is being used.
        """
        model = self.models[model_name]

        # Create a CVObject publisher for each class
        publisher_dict = {}
        for model_class in model['classes']:
            publisher_name = f"cv/{self.camera}/{model_class}"
            publisher_dict[model_class] = rospy.Publisher(publisher_name,
                                                          CVObject,
                                                          queue_size=10)
        self.publishers = publisher_dict

        if self.rgb_detections:
            self.detection_feed_publisher = rospy.Publisher("cv/front/detections/compressed", CompressedImage,
                                                            queue_size=10)

    def init_queues(self, device):
        """
        Assigns output queues from the pipeline to dictionary of queues.
        :param device: DepthAI.Device object for the connected device.
        See https://docs.luxonis.com/projects/api/en/latest/components/device/
        """
        # If the output queues are already set, don't reinitialize
        if self.connected:
            return

        self.output_queues["detections"] = device.getOutputQueue(name="detections", maxSize=1, blocking=False)
        self.output_queues["passthrough"] = device.getOutputQueue(
            name="feed", maxSize=1, blocking=False)

        self.input_queue = device.getInputQueue(name="nn_input", maxSize=1, blocking=False)

        self.connected = True  # Flag that the output queues have been initialized

        self.detection_visualizer = DetectionVisualizer(self.classes, self.colors,
                                                        self.show_class_name, self.show_confidence)

    def detect(self):
        """
        Get current detections from output queues and publish.
        """
        # init_output_queues must be called before detect
        if not self.connected:
            rospy.logwarn("Output queues are not initialized so cannot detect. Call init_output_queues first.")
            return

        # Get detections from output queues
        inDet = self.output_queues["detections"].tryGet()
        if not inDet:
            return
        detections = inDet.detections

        detections_dict = {}
        for detection in detections:
            prev_conf, prev_detection = detections_dict.get(detection.label, (None, None))
            if (prev_conf is not None and detection.confidence > prev_conf) or prev_conf is None:
                detections_dict[detection.label] = detection.confidence, detection

        detections = [detection for _, detection in detections_dict.values()]
        model = self.models[self.current_model_name]

        # Publish detections feed
        passthrough_feed = self.output_queues["passthrough"].tryGet()
        if not passthrough_feed:
            return
        frame = passthrough_feed.getCvFrame()

        if self.rgb_detections:
            detections_visualized = self.detection_visualizer.visualize_detections(frame, detections)
            detections_img_msg = self.image_tools.convert_to_ros_compressed_msg(detections_visualized)
            self.detection_feed_publisher.publish(detections_img_msg)

        # Process and publish detections. If using sonar, override det robot x coordinate
        for detection in detections:

            # Bounding box
            bbox = (detection.xmin, detection.ymin,
                    detection.xmax, detection.ymax)

            # Label name
            label_idx = detection.label
            label = self.classes[label_idx]

            confidence = detection.confidence

            # Calculate relative pose
            det_coords_robot_mm = calculate_relative_pose(bbox, model['input_size'], model['sizes'][label],
                                                          FOCAL_LENGTH, SENSOR_SIZE, 0.814)

            # Find yaw angle offset
            left_end_compute = self.compute_angle_from_x_offset(detection.xmin * self.camera_pixel_width)
            right_end_compute = self.compute_angle_from_x_offset(detection.xmax * self.camera_pixel_width)
            midpoint = (left_end_compute + right_end_compute) / 2.0
            yaw_offset = -math.radians(midpoint)  # Degrees to radians

            self.publish_prediction(
                bbox, det_coords_robot_mm, yaw_offset, label, confidence,
                (self.camera_pixel_height, self.camera_pixel_width), self.using_sonar)

    def publish_prediction(self, bbox, det_coords, yaw, label, confidence,
                           shape, using_sonar):
        """
        Publish predictions to label-specific topic. Publishes to /cv/[camera]/[label].

        :param bbox: Tuple for the bounding box.
            Values are from 0-1, where X increases left to right and Y increases top to bottom.
        :param det_coords: Tuple with the X, Y, and Z values in meters, and in the robot rotational reference frame.
        :param label: Predicted label for the detection.
        :param confidence: Confidence for the detection, from 0 to 1.
        :param shape: Tuple with the (height, width) of the image. NOTE: This is in reverse order from the model.
        """
        object_msg = CVObject()

        object_msg.header.stamp.secs = rospy.Time.now().secs
        object_msg.header.stamp.nsecs = rospy.Time.now().nsecs

        object_msg.label = label
        object_msg.score = confidence

        object_msg.coords.x = det_coords[0]
        object_msg.coords.y = det_coords[1]
        object_msg.coords.z = det_coords[2]

        object_msg.xmin = bbox[0]
        object_msg.ymin = bbox[1]
        object_msg.xmax = bbox[2]
        object_msg.ymax = bbox[3]

        object_msg.yaw = yaw

        object_msg.height = shape[0]
        object_msg.width = shape[1]

        object_msg.sonar = using_sonar

        if self.publishers:
            # Flush out 0, 0, 0 values
            # if object_msg.coords.x != 0 and object_msg.coords.y != 0 and object_msg.coords.z != 0:
            self.publishers[label].publish(object_msg)

    def update_sonar(self, sonar_results):
        """
        Callback function for listenting to sonar response
        Updates instance variable self.sonar_response based on
        what sonar throws back if it is in range (> SONAR_RANGE = 1.75m)
        """
        # Check to see if the sonar is in range - are results from sonar valid?
        if not sonar_results.is_object:
            return
        if sonar_results.pose.position.x > SONAR_RANGE and sonar_results.pose.position.x <= SONAR_DEPTH:
            self.in_sonar_range = True
            self.sonar_response = (sonar_results.pose.position.x, sonar_results.pose.position.y)
        else:
            self.in_sonar_range = False

    def update_priority(self, object):
        """
        Update the current priority class. If the priority class is detected, sonar will be called.
        """
        self.current_priority = object

    def run(self):
        """
        Runs the selected model on the connected device.
        :return: False if the model is not in cv/models/depthai_models.yaml.
        Otherwise, the model will be run on the device.
        """
        # Check if model is valid
        if self.running_model not in self.models:
            return False

        # Setup pipeline and publishers
        self.init_model(self.running_model)
        self.init_publishers(self.running_model)

        with depthai_camera_connect.connect(self.pipeline) as device:
            rospy.loginfo("Connected to DepthAI device")
            self.device = device
            self.init_queues(device)

            while not rospy.is_shutdown():
                self.detect()

        return True

    def compute_angle_from_x_offset(self, x_offset):
        """
        See: https://math.stackexchange.com/questions/1320285/convert-a-pixel-displacement-to-angular-rotation
        for implementation details.

        :param x_offset: x pixels from center of image

        :return: angle in degrees
        """
        image_center_x = self.camera_pixel_width / 2.0
        return math.degrees(math.atan(((x_offset - image_center_x) * 0.005246675486)))


if __name__ == '__main__':
    DepthAISpatialDetector().run()
