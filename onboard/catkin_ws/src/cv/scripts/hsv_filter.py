#!/usr/bin/env python3

import rospy
import cv2
import math
import resource_retriever as rr
import yaml

from copy import deepcopy
from opencv_apps.msg import Circle, RotatedRect
from sensor_msgs.msg import CompressedImage
from image_tools import ImageTools
from utils import DetectionVisualizer
from custom_msgs.srv import EnableModel
from custom_msgs.msg import CVObject
from image_tools import ImageTools


CAMERAS_FILEPATH = 'package://cv/configs/cameras.yaml'
HSV_MODELS_FILEPATH = 'package://cv/configs/hsv_models.yaml'
# TODO create and populate based on what we're reading from depth AI
# DONE utils.py update visualize_detections fn
# DONE create yaml file, top level will be each model (bin, buoy, etc.)


class HSVFilter:
    # Fitting functions for ellipse (e.g. path marker), rotated rectangle (e.g. bins), and circle (e.g. buoy).
    # See https://docs.opencv.org/4.x/dd/d49/tutorial_py_contour_features.html
    # For example, see detect_path_marker below for ellipse fitting.
    # Each function should return a dictionary of detection data, as well as a ROS Message for publishing.
    # You may utilize Circle and RotatedRect message types from opencv_apps:
    # https://docs.ros.org/en/kinetic/api/opencv_apps/html/index-msg.html

    # First define fitting functions so we can have them as values in a dict
    def fit_ellipse(self, contours, width, height):
        largest_contour = max(contours, key=cv2.contourArea)
        center, dimensions, orientation = cv2.fitEllipse(largest_contour)

        center_x = center[0] / width
        center_y = center[1] / height

        orientation_in_radians = math.radians(-orientation)

        ellipse_msg = RotatedRect()

        ellipse_msg.center.x = center_x
        ellipse_msg.center.y = center_y
        ellipse_msg.angle = orientation_in_radians
        ellipse_msg.size.width = dimensions[0]
        ellipse_msg.size.height = dimensions[1]

        return {"center": center, "dimensions": dimensions, "orientation": orientation_in_radians,
                "xmin": center[0] - (dimensions[0] / 2), "xmax": center[0] + (dimensions[0] / 2),
                "ymin": center[1] - (dimensions[1] / 2), "ymax": center[1] + (dimensions[1] / 2),
                "shape": "ellipse", "confidence": 1}, ellipse_msg

    def fit_circle(self, contours, width, height):
        largest_contour = max(contours, key=cv2.contourArea)
        center, radius = cv2.minEnclosingCircle(largest_contour)

        circle_msg = Circle()

        circle_msg.center.x = center[0]
        circle_msg.center.y = center[1]
        circle_msg.radius = radius

        return {"center": center, "radius": radius, "xmin": center[0] - radius, "xmax": center[0] + radius,
                "ymin": center[1] - radius, "ymax": center[1] + radius, "shape": "circle", "confidence": 1}, circle_msg

    def fit_rotated_rectangle(self, contours, width, height):
        largest_contour = max(contours, key=cv2.contourArea)
        center, dimensions, orientation = cv2.minAreaRect(largest_contour)

        rotated_rect_msg = RotatedRect()

        orientation_in_radians = math.radians(-orientation)

        rotated_rect_msg.center.x = center[0]
        rotated_rect_msg.center.y = center[1]
        rotated_rect_msg.angle = orientation_in_radians
        rotated_rect_msg.size.width = dimensions[0]
        rotated_rect_msg.size.height = dimensions[1]

        return {"center": center, "dimensions": dimensions, "orientation": orientation_in_radians,
                "xmin": center[0] - (dimensions[0] / 2), "xmax": center[0] + (dimensions[0] / 2),
                "ymin": center[1] - (dimensions[1] / 2), "ymax": center[1] + (dimensions[1] / 2),
                "shape": "rotated_rect", "confidence": 1}, rotated_rect_msg

    # Fitting functions and associated message types
    FITTING_FUNCTIONS = {
        "ellipse": fit_ellipse,
        "circle": fit_circle,
        "rotated_rectangle": fit_rotated_rectangle
    }

    MESSAGE_TYPES = {
        "ellipse": RotatedRect,
        "circle": Circle,
        "rotated_rectangle": RotatedRect
    }

    # Load in models and other misc. setup work
    def __init__(self):
        rospy.init_node('hsv_filter', anonymous=True)

        # - Read in cameras.yaml and hsv_models.yaml and create a dict of cameras (see depthai_spatial_detection.py).
        # For example:
        # { "depthai_front": {
        #     "topic": "/path",
        #     "models": [{
        #         "msg_type": messagetype,
        #         "upper_mask": [0, 0, 0],
        #         "lower_mask": [1, 1, 1],
        #         "shape": detect_ellipse,
        #         "visualizer_color": some_color,
        #         "enabled": False
        #     }]
        # }}
        #
        # Each camera should be initialized with ALL the models, initially set enabled to False.
        # - Also create a self.publishers dict containing publisher for each model (see init_publishers).
        # For publishing topic names, follow naming conventions in init_publishers function
        # - Create all necessary subscribers, one for each camera, with the same callback function self.detect.
        # Also pass in an extra camera argument so self.detect knows which camera subscriber it's being called from.
        # https://www.minsung.org/2018/07/passing-arguments-to-python-callback-in-rospy-ros/

        with open(rr.get_filename(CAMERAS_FILEPATH, use_protocol=False)) as f:
            cameras = yaml.safe_load(f)
        with open(rr.get_filename(HSV_MODELS_FILEPATH, use_protocol=False)) as f:
            models = yaml.safe_load(f)

        self.cameras_dict = {}
        self.publishers = {}
        self.subscribers = {}

        # for model, model_data in models.items():
        for model in models:
            models[model]["enabled"] = False
            # model_data["enabled"] = False
            # models[model] = model_data
            self.publishers[model] = rospy.Publisher(f"cv/{model}", CVObject, queue_size=10)

        for camera in cameras:
            # TODO add path to publish in cameras yaml file, talk with Vedarsh
            self.cameras_dict[camera] = {}
            self.cameras_dict[camera]["models"] = deepcopy(models)
            self.cameras_dict[camera]["publisher"] = rospy.Publisher(f"cv/{camera}/detections/compressed",
                                                                     CompressedImage, queue_size=10)
            # self.subscribers[camera] = rospy.Subscriber(f"cv/{camera}", CompressedImage, self.detect, (camera,))
            self.subscribers[camera] = rospy.Subscriber(f"camera/{camera}/rgb/preview/compressed", CompressedImage, self.detect, camera)
            # self.subscribers[camera] = rospy.Subscriber(f"cv/{camera}", CompressedImage, self.detect, (camera,))

            # TODO first param put path in camera yaml file

        # - Also create a service to enable toggling of models. It calls self.toggle_service as a callback function.
        # see detection.py for details
        rospy.Service('enable_model', EnableModel, self.toggle_service)

        self.image_tools = ImageTools()

        self.detection_visualizer = DetectionVisualizer(["bin", "collection_bin", "path_marker", "buoy"],
                                                        ["FF00000", "FF000000", "FF00000", "FF00000"])
        
        self.image_tools = ImageTools()


    def detect(self, data, camera):
        """Compute predictions on a raw image frame and publish results.

        This can be used as the camera topic subscriber callback.

        :param img_msg: ROS Image message to compute predictions on.
        """
        # This function should take in the camera and loop through all of its models.
        # For each model, if it is enabled:
        # - Initialize a map of {class: detections}
        # - Call find_contours to get the contour
        # - Call the corresponding fitting function, which returns detections and ROS Message to publish.
        # Add the detections to the map. Also uses the corresponding publisher to publish the ROS Message
        # Before publishing, check if message is not none and is of the type specified
        # - Pass the map into detection_visualizer, get the detections_visualized image
        # and publish (see code below)

        # detections_visualized = visualize_path_marker_detection(frame, detection)
        # detections_img_msg = self.image_tools.convert_to_ros_compressed_msg(detections_visualized)
        # self.detection_feed_publisher.publish(detections_img_msg)
        # DONE change this fn to something similar to depthai spatial detection line 347
        # i.e., create our own detect visualization fn
        detections = []
        camera_models = self.cameras_dict[camera]["models"]
        print(camera)
        print()
        for model in camera_models:
            # print(model["enabled"])
            if camera_models[model]["enabled"]:
                contour = self.find_contours(data, model.lower_mask, model.upper_mask)
                detection, message = self.FITTING_FUNCTIONS[model.shape](contour, data.width, data.height)
                detection["label"] = model.shape
                detections.append(detection)

                if (message is not None) and (isinstance(message, self.MESSAGE_TYPES[model.shape])):
                    self.publishers[model].publish(message)

        # Publishes frame with many detections on it (STC)
        current_image = self.image_tools.convert_to_cv2(data)
        detections_visualized = self.detection_visualizer.visualize_detections(current_image, detections)
        detections_img_msg = self.image_tools.convert_to_ros_compressed_msg(detections_visualized)
        self.cameras_dict[camera]["publisher"].publish(detections_img_msg)

    def find_contours(self, image, lower_mask, upper_mask):
        # Convert image to HSV
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        orange_mask = cv2.inRange(hsv_image, lower_mask, upper_mask)

        # Apply Gaussian blurring
        blurred_mask = cv2.GaussianBlur(orange_mask, (5, 5), 0)

        # Find contours in the mask
        contours, _ = cv2.findContours(blurred_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        return contours

    def toggle_service(self, data):
        # Each ROSService message will contain the camera, model, and enabled (True or False).
        # Toggle the "enabled" field of the corresponding model using the camera dict.
        # See run function in detection.py for Service syntax
        if data.camera in self.cameras_dict:
            if data.model_name in self.cameras_dict[data.camera]["models"]:
                self.cameras_dict[data.camera]["models"][data.model_name]["enabled"] = data.enabled
                return {"success": True, "message": f"""Successfully set enabled to {data.enabled} for {data.camera}
                        model {data.model_name}"""}
            else:
                return {"success": False, "message": f"""Could not set enabled to {data.enabled} for {data.camera}
                        model {data.model_name} | Camera does not have this model in the dictionary 'self.cameras_dict'
                        """}
        else:
            return {"success": False, "message": f"""Could not set enabled to {data.enabled} for {data.camera}
                    model {data.model_name} | Camera does exist in the dictionary 'self.cameras_dict'"""}

    def run(self):
        # while not rospy.is_shutdown():
        #     self.detect()

        # Keep node running until shut down
        rospy.spin()


if __name__ == '__main__':
    try:
        HSVFilter().run()
    except rospy.ROSInterruptException:
        pass
