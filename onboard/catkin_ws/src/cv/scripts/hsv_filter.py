#!/usr/bin/env python3

import rospy
import rostopic
import cv2
import depthai as dai
import math
import depthai_camera_connect
import numpy as np

from opencv_apps.msg import Circle, RotatedRect
from sensor_msgs.msg import CompressedImage
from image_tools import ImageTools
from utils import visualize_path_marker_detection


class HSVFilter:
    FITTING_FUNCTIONS = {
        # TODO: create mapping from object shape to fitting function used
    }

    # Load in models and other misc. setup work
    def __init__(self):
        rospy.init_node('hsv_filter', anonymous=True)

        # TODO:
        # - Read in cameras.yaml and hsv_models.yaml and create a dict of cameras (see depthai_spatial_detection.py).
        # For example:
        # { "depthai_front": {
        #     "topic": "/path",
        #     "models": [{
        #         "upper_mask": [0, 0, 0],
        #         "lower_mask": [1, 1, 1],
        #         "shape": detect_ellipse,
        #         "visualizer_color": some_color,
        #         "enabled": False
        #     }]
        # }}
        # Each camera should be initialized with ALL the models, initially set enabled to False.

        # - Also create a self.publishers dict containing publisher for each model (see init_publishers).
        # For publishing topic names, follow naming conventions in init_publishers function
        # - Create all necessary subscribers, one for each camera, with the same callback function self.detect.
        # Also pass in an extra camera argument so self.detect knows which camera subscriber it's being called from.
        # https://www.minsung.org/2018/07/passing-arguments-to-python-callback-in-rospy-ros/
        # - Also create a service to enable toggling of models. It calls self.toggle_service as a callback function.

        self.image_tools = ImageTools()

    def detect(self, data, camera):
        """Compute predictions on a raw image frame and publish results.

        This can be used as the camera topic subscriber callback.

        :param img_msg: ROS Image message to compute predictions on.
        """
        # TODO: this function should take in the camera and loop through all of its models.
        # For each model, if it is enabled:
        # - Initialize a map of {class: detections}
        # - Call find_contours to get the contour
        # - Call the corresponding fitting function, which returns detections and ROS Message to publish.
        # Add the detections to the map. Also uses the corresponding publisher to publish the ROS Message
        # - Pass the map into detection_visualizer, get the detections_visualized image
        # and publish (see code below)

        # detections_visualized = visualize_path_marker_detection(frame, detection)
        # detections_img_msg = self.image_tools.convert_to_ros_compressed_msg(detections_visualized)
        # self.detection_feed_publisher.publish(detections_img_msg)

    def find_contours(self, image, lower_mask, upper_mask):
        # TODO: write function to extract and return the contour from given image (see detect_path_marker below)
        pass

    # TODO: write fitting functions for ellipse (e.g. path marker), rotated rectangle (e.g. bins), and circle (e.g. buoy).
    # See https://docs.opencv.org/4.x/dd/d49/tutorial_py_contour_features.html
    # For example, see detect_path_marker below for ellipse fitting.
    # Each function should return a dictionary of detection data, as well as a ROS Message for publishing.
    # You may utilize Circle and RotatedRect message types from opencv_apps:
    # https://docs.ros.org/en/kinetic/api/opencv_apps/html/index-msg.html
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

            path_marker_msg = Rotated2DObject()

            path_marker_msg.center.x = center_x
            path_marker_msg.center.y = center_y
            path_marker_msg.angle = orientation_in_radians

            self.detection_publisher.publish(path_marker_msg)

            return {"center": center, "dimensions": dimensions, "orientation": orientation_in_radians}

    def toggle_service(self, data):
        # TODO: each ROSService message will contain the camera, model, and enabled (True or False).
        # Toggle the "enabled" field of the corresponding model using the camera dict.
        # See run function in detection.py for Service syntax
        pass

    def run(self):
        while not rospy.is_shutdown():
            self.detect()

        # Keep node running until shut down
        rospy.spin()


if __name__ == '__main__':
    try:
        HSVFilter().run()
    except rospy.ROSInterruptException:
        pass
