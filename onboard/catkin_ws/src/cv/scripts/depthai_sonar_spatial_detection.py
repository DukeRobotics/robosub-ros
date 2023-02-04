#!/usr/bin/env python3

import rospy
import resource_retriever as rr
import yaml
import cv2
import depthai_camera_connect
import depthai as dai
import numpy as np
from utils import DetectionVisualizer
from cv_bridge import CvBridge
from sonar_action_client import SonarClient
import math

from custom_msgs.srv import EnableModel
from custom_msgs.msg import CVObject
from sensor_msgs.msg import Image


MM_IN_METER = 1000
DEPTHAI_OBJECT_DETECTION_MODELS_FILEPATH = 'package://cv/models/depthai_models.yaml'
HORIZONTAL_FOV = 95
CAMERA_PIXEL_WIDTH = 416
SONAR_DEPTH = 5


# Compute detections on live camera feed and publish spatial coordinates for detected objects
class DepthAISpatialDetector:
    def __init__(self):
        """
        Initializes the ROS node and service. Loads the yaml file at cv/models/depthai_models.yaml
        """
        rospy.init_node('depthai_sonar_spatial_detection', anonymous=True)

        with open(rr.get_filename(DEPTHAI_OBJECT_DETECTION_MODELS_FILEPATH,
                                  use_protocol=False)) as f:
            self.models = yaml.safe_load(f)

        self.feed_path = rospy.get_param("~feed_path")
        self.latest_img = None

        # No feed path is passed in -- throw an expcetion
        if self.feed_path == "":
            rospy.logerr("No feed path variable given")
            rospy.spin()
            self.img = self._load_image_from_feed_path()

        self.camera = 'front'
        self.pipeline = None
        self.publishers = None
        self.output_queues = {}
        self.connected = False
        self.current_model_name = None
        self.classes = None
        self.detection_feed_publisher = None
        self.rgb_preview_publisher = None
        self.detection_visualizer = None

        self.enable_service = f'enable_model_{self.camera}'

        self.bridge = CvBridge()

        try:
            # Initializes a rospy node so that the SimpleActionClient can
            # publish and subscribe over ROS.
            self.sonar_client = SonarClient()
            self.sonar_client.sweep_at_center_angle(200, 2)
        except rospy.ROSInterruptException:
            rospy.loginfo("Node instantiation interrupted")

        rospy.loginfo("debug 1")

    def build_pipeline(self, nn_blob_path, sync_nn=True):
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
            - "boundingBoxDepthMapping": contains SpatialLocationCalculatorConfig messages, which provide a mapping
                                         between the RGB feed from which bounding boxes are computed and the depth map.
            - "depth": contains ImgFrame messages with UINT16 values representing the depth in millimeters by default.
                       See the depth of https://docs.luxonis.com/projects/api/en/latest/components/nodes/stereo_depth/

        :param nn_blob_path: Path to blob file used for object detection.
        :param sync_nn: If True, sync the RGB output feed with the detection from the neural network. Needed if the RGB
        feed output will be used and needs to be synced with the object detections.
        :return: depthai.Pipeline object to compute
        """
        model = self.models[self.current_model_name]

        pipeline = dai.Pipeline()

        # Define sources and outputs
        cam_rgb = pipeline.create(dai.node.ColorCamera)
        spatial_detection_network = pipeline.create(
            dai.node.YoloSpatialDetectionNetwork)
        mono_left = pipeline.create(dai.node.MonoCamera)
        mono_right = pipeline.create(dai.node.MonoCamera)
        stereo = pipeline.create(dai.node.StereoDepth)

        xout_rgb = pipeline.create(dai.node.XLinkOut)
        xout_nn = pipeline.create(dai.node.XLinkOut)
        xout_bounding_box_depth_mapping = pipeline.create(dai.node.XLinkOut)
        xout_depth = pipeline.create(dai.node.XLinkOut)

        if self.feed_path != "":
            # Point xIn to still image
            xout_rgb.setStreamName("stillImg")
        else:
            xout_rgb.setStreamName("rgb")
        
        xout_nn.setStreamName("detections")
        xout_bounding_box_depth_mapping.setStreamName(
            "boundingBoxDepthMapping")
        xout_depth.setStreamName("depth")

        # Properties
        cam_rgb.setPreviewSize(model['input_size'])
        cam_rgb.setResolution(
            dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam_rgb.setInterleaved(False)
        cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

        mono_left.setResolution(
            dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
        mono_right.setResolution(
            dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)

        # setting node configs
        stereo.setDefaultProfilePreset(
            dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        stereo.setDepthAlign(dai.CameraBoardSocket.RGB)

        spatial_detection_network.setBlobPath(nn_blob_path)
        spatial_detection_network.setConfidenceThreshold(0.5)
        spatial_detection_network.input.setBlocking(False)
        spatial_detection_network.setBoundingBoxScaleFactor(0.5)
        spatial_detection_network.setDepthLowerThreshold(100)
        spatial_detection_network.setDepthUpperThreshold(5000)

        # Yolo specific parameters
        spatial_detection_network.setNumClasses(len(model['classes']))
        spatial_detection_network.setCoordinateSize(model['coordinate_size'])
        spatial_detection_network.setAnchors(np.array(model['anchors']))
        spatial_detection_network.setAnchorMasks(model['anchor_masks'])
        spatial_detection_network.setIouThreshold(0.5)

        # Linking
        mono_left.out.link(stereo.left)
        mono_right.out.link(stereo.right)

        cam_rgb.preview.link(spatial_detection_network.input)
        if sync_nn:
            spatial_detection_network.passthrough.link(xout_rgb.input)
        else:
            cam_rgb.preview.link(xout_rgb.input)

        spatial_detection_network.out.link(xout_nn.input)
        spatial_detection_network.boundingBoxMapping.link(
            xout_bounding_box_depth_mapping.input)

        stereo.depth.link(spatial_detection_network.inputDepth)
        spatial_detection_network.passthroughDepth.link(xout_depth.input)

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

        Then model name is gate.
        """
        if model_name == self.current_model_name:
            return

        self.current_model_name = model_name

        model = self.models[model_name]

        self.classes = model['classes']

        blob_path = rr.get_filename(f"package://cv/models/{model['weights']}",
                                    use_protocol=False)
        self.pipeline = self.build_pipeline(blob_path)

    def init_publishers(self, model_name):
        """
        Initialize the publishers for the node. A publisher is created for each class that the model predicts.
        The publishers are created in format: "cv/camera/model_name".
        :param model_name: Name of the model that is being used.
        """
        model = self.models[model_name]
        publisher_dict = {}
        for model_class in model['classes']:
            publisher_name = f"cv/{self.camera}/{model_class}"
            publisher_dict[model_class] = rospy.Publisher(publisher_name,
                                                          CVObject,
                                                          queue_size=10)
        self.publishers = publisher_dict

        self.rgb_preview_publisher = rospy.Publisher(
            "camera/front/rgb/preview/stream_raw", Image, queue_size=10)
        self.detection_feed_publisher = rospy.Publisher(
            "cv/front/detections", Image, queue_size=10)

    def init_output_queues(self, device):
        """
        Assigns output queues from the pipeline to dictionary of queues.
        :param device: DepthAI.Device object for the connected device.
        See https://docs.luxonis.com/projects/api/en/latest/components/device/
        """
        if self.connected:
            return

        self.output_queues["rgb"] = device.getOutputQueue(
            name="rgb", maxSize=1, blocking=False)
        self.output_queues["detections"] = device.getOutputQueue(
            name="detections", maxSize=1, blocking=False)
        self.output_queues["boundingBoxDepthMapping"] = device.getOutputQueue(name="boundingBoxDepthMapping", maxSize=1,
                                                                              blocking=False)
        self.output_queues["depth"] = device.getOutputQueue(
            name="depth", maxSize=1, blocking=False)
        self.connected = True

        self.detection_visualizer = DetectionVisualizer(self.classes)

    def detect(self):
        """
        Get current detections from output queues and publish.
        """
        # Upload the pipeline to the device
        def to_planar(arr: np.ndarray, shape: tuple) -> np.ndarray:
            return cv2.resize(arr, shape).transpose(2, 0, 1).flatten()

        if not self.connected:
            return

        inPreview = None
        inDet = None

        if self.feed_path != "":

            inPreview = self.device.getInputQueue("stillImg")
            inDet = self.output_queues["detections"].get()

            iimg = dai.ImgFrame()
            iimg.setType(dai.ImgFrame.Type.BGR888p)
            iimg.setData(to_planar(self.img, (416, 416)))
            iimg.setWidth(self.model['input_size'][0])
            iimg.setHeight(self.model['input_size'][1])
            inPreview.send(iimg)

        else:
            inPreview = self.output_queues["rgb"].get()
            inDet = self.output_queues["detections"].get()
        
        frame = inPreview.getCvFrame()
        detections = inDet.detections

        frame_img_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        self.rgb_preview_publisher.publish(frame_img_msg)

        rospy.loginfo("debug")

        detections_img_msg = self.bridge.cv2_to_imgmsg(
            self.detection_visualizer.visualize_detections(frame, detections),
            'bgr8'
        )
        self.detection_feed_publisher.publish(detections_img_msg)

        height = frame.shape[0]
        width = frame.shape[1]
        for detection in detections:

            bbox = (detection.xmin, detection.ymin,
                    detection.xmax, detection.ymax)

            label_idx = detection.label
            label = self.classes[label_idx]

            confidence = detection.confidence
            # x is left/right axis, where 0 is in middle of the frame, to the left is negative x,
            # and to the right is positive x
            # y is down/up axis, where 0 is in the middle of the frame, down is negative y, and up is positive y
            # z is distance of object from camera in mm
            x_cam_mm = detection.spatialCoordinates.x
            y_cam_mm = detection.spatialCoordinates.y
            z_cam_mm = detection.spatialCoordinates.z

            # Get sonar sweep range
            (left_end, right_end) = coords_to_angle(
                detection.xmin, detection.xmax)
            center = (left_end + right_end) / 2.0
            breadth = abs(center - left_end)

            x_cam_meters = mm_to_meters(x_cam_mm)
            y_cam_meters = mm_to_meters(y_cam_mm)
            z_cam_meters = mm_to_meters(z_cam_mm)
            using_sonar = False

            det_coords_robot_mm = camera_frame_to_robot_frame(
                    x_cam_meters, y_cam_meters, z_cam_meters)

            # Try calling sonar on detected bounding box
            # if sonar responds, then override existing robot-frame x, y info; else, keep default
            try:
                # Request sonar to sweep within bounded angle range; read center of mass of detected object from sonar
                result = self.sonar_client.sweep_at_center_angle(
                    center, breadth, SONAR_DEPTH)
                # Sonar gives robot x,y; camera gives camera y, which is robot z
                # Override det_coords_robot_mm with updated sonar data
                det_coords_robot_mm = (result.x_pos, result.y_pos, y_cam_meters)
                using_sonar = True
            except rospy.ROSInterruptException:
                rospy.loginfo("Sonar sweep failed, defaulting to stereo")

            self.publish_prediction(
                bbox, det_coords_robot_mm, label, confidence, (height, width), using_sonar)

    def publish_prediction(self, bbox, det_coords, label, confidence, shape, using_sonar):
        """
        Publish predictions to label-specific topic. Publishes to /cv/[camera]/[label].

        :param bbox: Tuple for the bounding box. Values are from 0-1, where X increases left to right and Y increases.
        :param det_coords: Tuple with the X, Y, and Z values in meters, and in the robot rotational reference frame.
        :param label: Predicted label for the detection.
        :param confidence: Confidence for the detection, from 0 to 1.
        :param shape: Tuple with the (height, width) of the image.
        :param using_sonar: Boolean representing whether or not data was retrieved from sonar.
        """
        object_msg = CVObject()
        object_msg.label = label
        object_msg.score = confidence

        object_msg.coords.x = det_coords[0]
        object_msg.coords.y = det_coords[1]
        object_msg.coords.z = det_coords[2]

        object_msg.xmin = bbox[0]
        object_msg.ymin = bbox[1]
        object_msg.xmax = bbox[2]
        object_msg.ymax = bbox[3]

        object_msg.height = shape[0]
        object_msg.width = shape[1]

        object_msg.sonar = using_sonar

        if self.publishers:
            self.publishers[label].publish(object_msg)

    def run_model(self, req):
        """
        Runs the model on the connected device.
        :param req: Request from
        :return: False if the model is not in cv/models/depthai_models.yaml.
        Otherwise, the model will be run on the device.
        """
        if req.model_name not in self.models:
            return False

        self.init_model(req.model_name)
        self.init_publishers(req.model_name)

        with depthai_camera_connect.connect(self.pipeline) as device:
            self.device = device
            self.init_output_queues(device)

            loop_rate = rospy.Rate(1)
            while not rospy.is_shutdown():
                self.detect()
                loop_rate.sleep()

        return True

    def run(self):
        """
        Wait for the EnableModel rosservice call. If this call is made, the model specified will be run.
        For information about EnableModel, see robosub-ros/core/catkin_ws/src/custom_msgs/srv/EnableModel.srv

        Example ros commands to run this node and activate the model:
            roslaunch cv spatial_detection_front.launch
            rosservice call /enable_model_front gate True

        """
        rospy.Service(self.enable_service, EnableModel, self.run_model)
        rospy.spin()

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
        """ Store latest image """
        self.latest_img = self.cv_bridge.imgmsg_to_cv2(img_msg, 'bgr8')


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


def coords_to_angle(min_x, max_x):
    """
    Takes in a detected bounding box from the camera and returns the angle range to sonar sweep.
    :param min_x: minimum x coordinate of camera bounding box (robot y)
    :param max_x: maximum x coordinate of camera bounding box (robot y)
    """
    distance_to_screen = CAMERA_PIXEL_WIDTH/2 * \
        1/math.tan(math.radians(HORIZONTAL_FOV/2))
    min_angle = math.degrees(np.arctan(min_x/distance_to_screen))
    max_angle = math.degrees(np.arctan(max_x/distance_to_screen))
    return min_angle, max_angle


if __name__ == '__main__':
    DepthAISpatialDetector().run()
