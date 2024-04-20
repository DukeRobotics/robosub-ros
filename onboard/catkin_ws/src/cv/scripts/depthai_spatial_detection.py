#!/usr/bin/env python3

import rospy
import resource_retriever as rr
import yaml
import math
import depthai_camera_connect
import depthai as dai
import numpy as np
from utils import DetectionVisualizer
from image_tools import ImageTools

from custom_msgs.msg import CVObject, SonarSweepRequest
from custom_msgs.srv import SetDepthAIModel
from geometry_msgs.msg import Pose
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from std_srvs.srv import SetBool


MM_IN_METER = 1000
DEPTHAI_OBJECT_DETECTION_MODELS_FILEPATH = 'package://cv/models/depthai_models.yaml'
DEPTHAI_SCRIPT_NODES_PATH = 'package://cv/scripts/depthai_script_nodes/'
HORIZONTAL_FOV = 95
SONAR_DEPTH = 10
SONAR_RANGE = 1.75
SONAR_REQUESTS_PATH = 'sonar/request'
SONAR_RESPONSES_PATH = 'sonar/cv/response'
TASK_PLANNING_REQUESTS_PATH = "controls/desired_feature"


# Compute detections on live camera feed and publish spatial coordinates for detected objects
class DepthAISpatialDetector:
    def __init__(self):
        """
        Initializes the ROS node. Loads the yaml file at cv/models/depthai_models.yaml
        """
        rospy.init_node('depthai_spatial_detection', anonymous=True)
        self.camera = rospy.get_param("~camera")
        self.initial_model_name = rospy.get_param("~model")
        self.rgb_raw = rospy.get_param("~rgb_raw")
        self.rgb_detections = rospy.get_param("~rgb_detections")
        self.queue_rgb = self.rgb_raw or self.rgb_detections  # Whether to output RGB feed
        self.queue_depth = rospy.get_param("~depth")  # Whether to output depth map
        self.sync_nn = rospy.get_param("~sync_nn")
        self.using_sonar = rospy.get_param("~using_sonar")
        self.show_class_name = rospy.get_param("~show_class_name")
        self.show_confidence = rospy.get_param("~show_confidence")

        with open(rr.get_filename(DEPTHAI_OBJECT_DETECTION_MODELS_FILEPATH, use_protocol=False)) as f:
            self.models = yaml.safe_load(f)

        # Pipeline
        self.pipeline = None

        # Publishers
        self.publishers = {}  # Keys are the class names of a given model
        self.detection_feed_publisher = None
        self.rgb_preview_publisher = None
        self.detection_visualizer = None

        # Input/Output queues
        self.input_queue_model = None
        self.output_queues = {}  # Keys are "rgb", "depth", and "detections"
        self.connected = False

        # Current DepthAI model
        self.current_model_name = None
        self.classes = None
        self.camera_pixel_width = None
        self.camera_pixel_height = None

        # ROS Services
        self.set_model_service = f'set_model_{self.camera}'
        self.enable_model_service = f'enable_model_{self.camera}'

        # Image tools
        self.image_tools = ImageTools()

        # Sonar
        self.sonar_response = (0, 0)
        self.in_sonar_range = True
        self.sonar_busy = False

        # By default the first task is going through the gate
        self.current_priority = "buoy_abydos_serpenscaput"

        # Initialize publishers and subscribers for sonar/task planning
        self.sonar_requests_publisher = rospy.Publisher(
            SONAR_REQUESTS_PATH, SonarSweepRequest, queue_size=10)
        self.sonar_response_subscriber = rospy.Subscriber(
            SONAR_RESPONSES_PATH, Pose, self.update_sonar)
        self.desired_detection_feature = rospy.Subscriber(
            TASK_PLANNING_REQUESTS_PATH, String, self.update_priority)

        # Enable detections
        self.detections_enabled = False

    def build_pipeline(self):
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

        # :param nn_blob_path: Path to blob file used for object detection.
        :param sync_nn: If True, sync the RGB output feed with the detection from the neural network. Needed if the RGB
        feed output will be used and needs to be synced with the object detections.
        :return: depthai.Pipeline object to compute
        """
        pipeline = dai.Pipeline()

        # Camera properties
        cam_rgb = pipeline.create(dai.node.ColorCamera)
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam_rgb.setInterleaved(False)
        cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        cam_rgb.setPreviewSize([416, 416])

        # Mono properties
        mono_left = pipeline.create(dai.node.MonoCamera)
        mono_right = pipeline.create(dai.node.MonoCamera)

        mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
        mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_right.setBoardSocket(dai.CameraBoardSocket.CAM_C)

        # Stereo properties
        stereo = pipeline.create(dai.node.StereoDepth)

        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)

        # Script nodes to handle switching models on the fly
        switch_model_in = pipeline.create(dai.node.Script)
        switch_model_in_path = DEPTHAI_SCRIPT_NODES_PATH + 'switch_model_in.py'
        with open(rr.get_filename(switch_model_in_path, use_protocol=False)) as f:
            switch_model_in.setScript(f.read())

        switch_model_out = pipeline.create(dai.node.Script)
        switch_model_out_path = DEPTHAI_SCRIPT_NODES_PATH + 'switch_model_out.py'
        with open(rr.get_filename(switch_model_out_path, use_protocol=False)) as f:
            switch_model_out.setScript(f"queue_depth = {self.queue_depth}\n\n{f.read()}")

        # Model input queue
        xin_model = pipeline.create(dai.node.XLinkIn)
        xin_model.setStreamName("model")

        # Output queues
        xout_nn = pipeline.create(dai.node.XLinkOut)
        xout_nn.setStreamName("detections")

        if self.queue_rgb:
            xout_rgb = pipeline.create(dai.node.XLinkOut)
            xout_rgb.setStreamName("rgb")

        if self.queue_depth:
            xout_depth = pipeline.create(dai.node.XLinkOut)
            xout_depth.setStreamName("depth")

        # Linking
        xin_model.out.link(switch_model_in.inputs["model"])
        xin_model.out.link(switch_model_out.inputs["model"])

        mono_left.out.link(stereo.left)
        mono_right.out.link(stereo.right)

        cam_rgb.preview.link(switch_model_in.inputs["input"])
        stereo.depth.link(switch_model_in.inputs["inputDepth"])

        switch_model_in.outputs["raw_input"].link(switch_model_out.inputs["raw_passthrough"])
        switch_model_in.outputs["raw_inputDepth"].link(switch_model_out.inputs["raw_passthroughDepth"])

        # Spatial detection network nodes
        for model_name in self.models:
            model = self.models[model_name]

            blob_path = rr.get_filename(f"package://cv/models/{model['weights']}", use_protocol=False)

            # Set up a new network for each model
            spatial_detection_network = pipeline.create(dai.node.YoloSpatialDetectionNetwork)

            spatial_detection_network.setBlobPath(blob_path)
            spatial_detection_network.setConfidenceThreshold(model['confidence_threshold'])
            spatial_detection_network.input.setBlocking(False)
            spatial_detection_network.setBoundingBoxScaleFactor(0.5)
            spatial_detection_network.setDepthLowerThreshold(100)
            spatial_detection_network.setDepthUpperThreshold(5000)

            # Yolo specific parameters
            spatial_detection_network.setNumClasses(len(model['classes']))
            spatial_detection_network.setCoordinateSize(model['coordinate_size'])
            spatial_detection_network.setAnchors(np.array(model['anchors']))
            spatial_detection_network.setAnchorMasks(model['anchor_masks'])
            spatial_detection_network.setIouThreshold(model['iou_threshold'])

            # Linking switch node outputs to spatial detection network inputs
            switch_model_in.outputs[f"{model['id']}_input"].link(spatial_detection_network.input)
            switch_model_in.outputs[f"{model['id']}_inputDepth"].link(spatial_detection_network.inputDepth)

            spatial_detection_network.passthrough.link(switch_model_out.inputs[f"{model['id']}_passthrough"])
            spatial_detection_network.passthroughDepth.link(switch_model_out.inputs[f"{model['id']}_passthroughDepth"])
            spatial_detection_network.out.link(switch_model_out.inputs[f"{model['id']}_out"])

            print(f"{model['id']}_passthrough")

        if self.queue_rgb:
            # To sync RGB frames with NN, link passthrough to xout instead of preview
            if self.sync_nn:
                switch_model_out.outputs["passthrough"].link(xout_rgb.input)
            else:
                cam_rgb.preview.link(xout_rgb.input)

        switch_model_out.outputs["out"].link(xout_nn.input)

        if self.queue_depth:
            switch_model_out.outputs["passthroughDepth"].link(xout_depth.input)

        self.pipeline = pipeline

    def init_model(self, model_name):
        """
        Sets the current model name and model attributes.

        :param model_name: Name of the model. The model name should match a key in cv/models/depthai_models.yaml.
        For example, if depthai_models.yaml is:

        gate:
            id: 1
            classes: ['gate', 'gate_side', 'gate_tick', 'gate_top', 'start_gate']
            topic: cv/
            weights: yolo_v4_tiny_openvino_2021.3_6shave-2022-7-21_416_416.blob
            input_size: [416, 416]
            coordinate_size: 4
            anchors: [10, 14, 23, 27, 37, 58, 81, 82, 135, 169, 344, 319]
            anchor_masks: {"side26": [0, 1, 2], "side13": [3, 4, 5]}

        Then, a possible model name is "gate".
        """
        self.current_model_name = model_name

        model = self.models[model_name]
        self.classes = model['classes']
        self.camera_pixel_width, self.camera_pixel_height = model['input_size']
        self.colors = model['colors']

        self.detection_visualizer = DetectionVisualizer(self.classes, self.colors,
                                                        self.show_class_name, self.show_confidence)

        # Send model name to the pipeline
        input_model_msg = dai.Buffer()
        input_model_msg.setData(model['id'])
        self.input_queue_model.send(input_model_msg)

    def init_publishers(self):
        """
        Initialize the publishers for the node. A publisher is created for each class that the model predicts.
        The publishers are created in format: "cv/camera/model_name".
        :param model_name: Name of the model that is being used.
        """
        # Unregister all current CVObject publishers and reset self.publishers
        for publisher in self.publishers.values():
            publisher.unregister()

        self.publishers = {}

        # (Re)create a CVObject publisher for each class
        model = self.models[self.current_model_name]
        for model_class in model['classes']:
            publisher_name = f"cv/{self.camera}/{model_class}"
            self.publishers[model_class] = rospy.Publisher(publisher_name, CVObject, queue_size=10)

        # Create CompressedImage publishers for the raw RGB feed, detections feed, and depth feed
        # Only create these publishers once
        if self.rgb_raw and not self.rgb_preview_publisher:
            self.rgb_preview_publisher = rospy.Publisher(f"camera/{self.camera}/rgb/preview/compressed",
                                                         CompressedImage, queue_size=10)

        if self.queue_depth and not self.depth_publisher:
            self.depth_publisher = rospy.Publisher(f"camera/{self.camera}/depth/compressed", CompressedImage,
                                                   queue_size=10)

        if self.rgb_detections and not self.detection_feed_publisher:
            self.detection_feed_publisher = rospy.Publisher(f"cv/{self.camera}/detections/compressed", CompressedImage,
                                                            queue_size=10)

    def init_device_queues(self, device):
        """
        Assigns output queues from the pipeline to dictionary of queues.
        :param device: DepthAI.Device object for the connected device.
        See https://docs.luxonis.com/projects/api/en/latest/components/device/
        """
        # If the output queues are already set, don't reinitialize
        if self.connected:
            return

        self.input_queue_model = device.getInputQueue("model")

        # Assign output queues
        if self.queue_rgb:
            self.output_queues["rgb"] = device.getOutputQueue(name="rgb", maxSize=1, blocking=False)

        if self.queue_depth:
            self.output_queues["depth"] = device.getOutputQueue(name="depth", maxSize=1, blocking=False)

        self.output_queues["detections"] = device.getOutputQueue(name="detections", maxSize=1, blocking=False)

        self.connected = True  # Flag that the output queues have been initialized

    def detect(self):
        """
        Get current detections from output queues and publish.
        """
        # init_output_queues must be called before detect
        if not self.connected:
            rospy.logwarn("Output queues are not initialized so cannot detect. Call init_output_queues first.")
            return

        if self.queue_rgb:
            inPreview = self.output_queues["rgb"].get()
            frame = inPreview.getCvFrame()

            if self.rgb_raw:
                # Publish raw RGB feed
                frame_img_msg = self.image_tools.convert_to_ros_compressed_msg(frame)
                self.rgb_preview_publisher.publish(frame_img_msg)

        # Publish depth feed
        if self.queue_depth:
            raw_img_depth = self.output_queues["depth"].get()
            img_depth = raw_img_depth.getCvFrame()
            image_msg_depth = self.image_tools.convert_depth_to_ros_compressed_msg(img_depth, 'mono16')
            self.depth_publisher.publish(image_msg_depth)

        # Get detections from output queues
        inDet = self.output_queues["detections"].tryGet()
        if inDet:
            detections = inDet.detections

            # Publish detections feed
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

                # x is left/right axis, where 0 is in middle of the frame,
                # to the left is negative x, and to the right is positive x
                # y is down/up axis, where 0 is in the middle of the frame,
                # down is negative y, and up is positive y
                # z is distance of object from camera in mm
                x_cam_mm = detection.spatialCoordinates.x
                y_cam_mm = detection.spatialCoordinates.y
                z_cam_mm = detection.spatialCoordinates.z

                x_cam_meters = mm_to_meters(x_cam_mm)
                y_cam_meters = mm_to_meters(y_cam_mm)
                z_cam_meters = mm_to_meters(z_cam_mm)

                det_coords_robot_mm = camera_frame_to_robot_frame(x_cam_meters, y_cam_meters, z_cam_meters)

                # Find yaw angle offset
                left_end_compute = self.compute_angle_from_x_offset(detection.xmin * self.camera_pixel_width)
                right_end_compute = self.compute_angle_from_x_offset(detection.xmax * self.camera_pixel_width)
                midpoint = (left_end_compute + right_end_compute) / 2.0
                yaw_offset = (midpoint) * (math.pi / 180.0)  # Degrees to radians

                # Create a new sonar request msg object if using sonar and the current detected
                # class is the desired class to be returned to task planning
                if self.using_sonar and label == self.current_priority:

                    # top_end_compute = self.compute_angle_from_y_offset(detection.ymin * self.camera_pixel_height)
                    # bottom_end_compute = self.compute_angle_from_y_offset(detection.ymax * self.camera_pixel_height)

                    # Construct sonar request message
                    sonar_request_msg = SonarSweepRequest()
                    sonar_request_msg.start_angle = int(left_end_compute)
                    sonar_request_msg.end_angle = int(right_end_compute)
                    sonar_request_msg.distance_of_scan = int(SONAR_DEPTH)

                    # Make a request to sonar if it is not busy
                    if not self.sonar_busy:
                        self.sonar_requests_publisher.publish(sonar_request_msg)
                        # Getting response...
                        self.sonar_busy = True

                    # Try calling sonar on detected bounding box
                    # if sonar responds, then override existing robot-frame x info;
                    # else, keep default
                    if not (self.sonar_response == (0, 0)) and self.in_sonar_range:
                        det_coords_robot_mm = (self.sonar_response[0],  # Override x
                                               -x_cam_meters,  # Maintain original y
                                               y_cam_meters)  # Maintain original z

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
            if object_msg.coords.x != 0 and object_msg.coords.y != 0 and object_msg.coords.z != 0:
                self.publishers[label].publish(object_msg)

    def update_sonar(self, sonar_results):
        """
        Callback function for listenting to sonar response
        Updates instance variable self.sonar_response based on
        what sonar throws back if it is in range (> SONAR_RANGE = 1.75m)
        """
        # Check to see if the sonar is in range - are results from sonar valid?
        self.sonar_busy = False
        if sonar_results.position.x > SONAR_RANGE and sonar_results.position.x <= SONAR_DEPTH:
            self.in_sonar_range = True
            self.sonar_response = (sonar_results.position.x, sonar_results.position.y)
        else:
            self.in_sonar_range = False

    def update_priority(self, object):
        """
        Update the current priority class. If the priority class is detected, sonar will be called.
        """
        self.current_priority = object

    def set_model(self, req):
        new_model_name = req.model_name

        if new_model_name == self.current_model_name:
            return {"success": True, "message": f"Model {new_model_name} is already running on {self.camera} camera."}

        if new_model_name not in self.models:
            return {"success": False, "message": f"FAILURE: {new_model_name} is not a valid model name."}

        else:
            self.detections_enabled = False

            self.init_model(new_model_name)
            self.init_publishers(new_model_name)

            self.detections_enabled = True

            return {"success": True, "message": f"Sucessfully set new model {new_model_name} for {self.camera} camera."}

    def enable_model(self, req):
        """Service for toggling specific models on and off.

        :param req: The request from another node or command line to enable the model.
        """
        input_model_msg = dai.Buffer()
        if req.data:
            input_model_msg.setData(self.models[self.current_model_name]['id'])
            message = "Successfully enabled model."
        else:
            input_model_msg.setData(0)
            message = "Successfully disabled model."
        self.input_queue_model.send(input_model_msg)

        return {"success": True, "message": message}

    def run(self):
        """
        Runs the selected model on the connected device.
        :return: False if the model is not in cv/models/depthai_models.yaml.
        Otherwise, the model will be run on the device.
        """
        # Check if model is valid
        if self.initial_model_name not in self.models:
            return False

        # Allow service for toggling of models
        rospy.Service(self.set_model_service, SetDepthAIModel, self.set_model)
        rospy.Service(self.enable_model_service, SetBool, self.enable_model)

        # Setup pipeline and publishers
        self.build_pipeline()

        with depthai_camera_connect.connect(self.pipeline, self.camera) as device:
            self.init_device_queues(device)
            self.init_model(self.initial_model_name)
            self.init_publishers()

            self.detections_enabled = True

            while not rospy.is_shutdown() and self.detections_enabled:
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

    def compute_angle_from_y_offset(self, y_offset):
        """
        See: https://math.stackexchange.com/questions/1320285/convert-a-pixel-displacement-to-angular-rotation
        for implementation details.

        :param y_offset: y pixels from center of image

        :return: angle in degrees
        """
        image_center_y = self.camera_pixel_height / 2.0
        return math.degrees(math.atan(((y_offset - image_center_y) * 0.003366382395)))

    def coords_to_angle(self, min_x, max_x):
        """
        Takes in a detected bounding box from the camera and returns the angle
                range to sonar sweep.
        :param min_x: minimum x coordinate of camera bounding box (robot y)
        :param max_x: maximum x coordinate of camera bounding box (robot y)
        """
        distance_to_screen = self.camera_pixel_width / 2 * \
            1/math.tan(math.radians(HORIZONTAL_FOV/2))
        min_angle = math.degrees(np.arctan(min_x/distance_to_screen))
        max_angle = math.degrees(np.arctan(max_x/distance_to_screen))
        return min_angle, max_angle


def mm_to_meters(val_mm):
    """
    Converts value from millimeters to meters.
    :param val_mm: Value in millimeters.
    :return: Input value converted to meters.
    """
    return val_mm / MM_IN_METER


def camera_frame_to_robot_frame(cam_x, cam_y, cam_z):
    """
    Convert coordinates in camera reference frame to coordinates in robot reference frame.
    This ONLY ACCOUNTS FOR THE ROTATION BETWEEN COORDINATE FRAMES, and DOES NOT ACCOUNT FOR THE TRANSLATION.
    :param cam_x: X coordinate of object in camera reference frame.
    :param cam_y: Y coordinate of object in camera reference frame.
    :param cam_z: Z coordinate of object in camera reference frame.
    :return: X,Y,Z coordinates of object in robot rotational reference frame.
    """
    robot_y = -cam_x
    robot_z = cam_y
    robot_x = cam_z
    return robot_x, robot_y, robot_z


if __name__ == '__main__':
    DepthAISpatialDetector().run()
