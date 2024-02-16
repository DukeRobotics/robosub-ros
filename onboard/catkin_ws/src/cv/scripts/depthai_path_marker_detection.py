#!/usr/bin/env python3

import rospy
import rostopic
import cv2
import depthai as dai
import math
import depthai_camera_connect
import numpy as np

from custom_msgs.msg import PathMarker
from sensor_msgs.msg import CompressedImage
from image_tools import ImageTools
from utils import visualize_path_marker_detection


class DepthAIPathMarkerDetector:
    """This class computes and publishes predictions on a image stream."""
    LOWER_ORANGE = np.array([5, 50, 50])
    UPPER_ORANGE = np.array([33, 255, 255])
    PATH_MARKER_PUBLISH_TOPIC = "cv/downward/path_marker"

    # Load in models and other misc. setup work
    def __init__(self):
        rospy.init_node('depthai_path_marker_detection', anonymous=True)
        self.rgb_raw = rospy.get_param("~rgb_raw")
        self.rgb_detections = rospy.get_param("~rgb_detections")
        self.queue_rgb = self.rgb_raw or self.rgb_detections  # Whether to output RGB feed

        self.image_tools = ImageTools()
        self.camera = "downwards"

        # The topic that the camera publishes its feed to
        self.camera_feed_topic = f'/camera/{self.camera}/compressed'
        self.publisher = rospy.Publisher(self.PATH_MARKER_PUBLISH_TOPIC, PathMarker, queue_size=10)

        self.pipeline = None
        self.build_pipeline()

        # Create CompressedImage publishers for the raw RGB feed, detections feed, and depth feed
        if self.rgb_raw:
            self.rgb_preview_publisher = rospy.Publisher(f"camera/{self.camera}/rgb/preview/compressed",
                                                         CompressedImage, queue_size=10)

        if self.rgb_detections:
            self.detection_feed_publisher = rospy.Publisher(f"cv/{self.camera}/detections/compressed", CompressedImage,
                                                            queue_size=10)

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
        The output queue is "rgb", which contains the 400x400 RGB preview of the camera feed.

        :return: depthai.Pipeline object to compute
        """
        pipeline = dai.Pipeline()

        cam_rgb = pipeline.create(dai.node.ColorCamera)
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam_rgb.setInterleaved(False)
        cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

        xout_rgb = pipeline.create(dai.node.XLinkOut)
        xout_rgb.setStreamName("rgb")

        cam_rgb.preview.link(xout_rgb.input)

        return pipeline

    def detect(self):
        """Compute predictions on a raw image frame and publish results.

        This can be used as the camera topic subscriber callback.

        :param img_msg: ROS Image message to compute predictions on.
        """

        if self.queue_rgb:
            inPreview = self.output_queues["rgb"].get()
            frame = inPreview.getCvFrame()

            detection = self.detect_path_marker(frame)

            # Publish raw RGB feed
            if self.rgb_raw:
                frame_img_msg = self.image_tools.convert_to_ros_compressed_msg(frame)
                self.rgb_preview_publisher.publish(frame_img_msg)

            # Publish detections feed
            if self.rgb_detections and detection:
                detections_visualized = visualize_path_marker_detection(frame, detection)
                detections_img_msg = self.image_tools.convert_to_ros_compressed_msg(detections_visualized)
                self.detection_feed_publisher.publish(detections_img_msg)

    def detect_path_marker(self, image):
        height, width, _ = image.shape

        # Convert image to HSV
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        orange_mask = cv2.inRange(hsv_image, self.LOWER_ORANGE, self.UPPER_ORANGE)

        # Apply Gaussian blurring
        blurred_mask = cv2.GaussianBlur(orange_mask, (5, 5), 0)

        # Find contours in the mask
        contours, _ = cv2.findContours(blurred_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Fit a line to the largest contour
        if len(contours) > 0:
            largest_contour = max(contours, key=cv2.contourArea)
            center, dimensions, orientation = cv2.fitEllipse(largest_contour)

            # Extract the center and orientation of the ellipse
            center_x = center[0] / width
            center_y = center[1] / height

            orientation_in_radians = math.radians(-orientation)

            path_marker_msg = PathMarker()

            path_marker_msg.x = center_x
            path_marker_msg.y = center_y
            path_marker_msg.angle = orientation_in_radians

            self.detection_publisher.publish(path_marker_msg)

            return {"center": center, "dimensions": dimensions, "orientation": orientation_in_radians}

    def run(self):
        """Initialize node and set up Subscriber to generate and publish predictions at every camera frame received."""
        TopicType, _, _ = rostopic.get_topic_class(self.camera_feed_topic)
        rospy.Subscriber(self.camera_feed_topic, TopicType, self.detect)

        depthai_camera_connect.connect(self.pipeline, self.camera)

        while not rospy.is_shutdown():
            self.detect()

        # Keep node running until shut down
        rospy.spin()


if __name__ == '__main__':
    try:
        DepthAIPathMarkerDetector().run()
    except rospy.ROSInterruptException:
        pass
