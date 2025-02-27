#!/usr/bin/env python3

import numpy as np
import math
import cv2
import os
from geometry_msgs.msg import Point
from custom_msgs.msg import CVObject


def check_file_writable(filepath):
    """
    Check if a file can be created or overwritten.
    """
    if os.path.exists(filepath):
        # path exists
        if os.path.isfile(filepath):
            # also works when file is a link and the target is writable
            return os.access(filepath, os.W_OK)
        else:
            # path is a dir, so cannot write as a file
            return False
    # target does not exist, check perms on parent dir
    pdir = os.path.dirname(filepath)
    if not pdir:
        pdir = '.'
    # target is creatable if parent dir is writable
    return os.access(pdir, os.W_OK)


def cam_dist_with_obj_width(width_pixels, width_meters,
                            focal_length, img_shape, sensor_size, adjustment_factor=1):
    '''
        Note that adjustment factor is 1 for mono camera and 2 for depthAI camera
    '''
    return (focal_length * width_meters * img_shape[0]) \
        / (width_pixels * sensor_size[0]) * adjustment_factor


def cam_dist_with_obj_height(height_pixels, height_meters,
                             focal_length, img_shape, sensor_size, adjustment_factor=1):
    return (focal_length * height_meters * img_shape[1]) \
        / (height_pixels * sensor_size[1]) * adjustment_factor


def compute_yaw(xmin, xmax, camera_pixel_width):
    # Find yaw angle offset
    left_end_compute = compute_angle_from_x_offset(xmin * camera_pixel_width, camera_pixel_width)
    right_end_compute = compute_angle_from_x_offset(xmax * camera_pixel_width, camera_pixel_width)
    midpoint = (left_end_compute + right_end_compute) / 2.0
    return (midpoint) * (math.pi / 180.0)  # Degrees to radians


def compute_angle_from_x_offset(x_offset, camera_pixel_width):
    """
    See: https://math.stackexchange.com/questions/1320285/convert-a-pixel-displacement-to-angular-rotation
    for implementation details.

    :param x_offset: x pixels from center of image

    :return: angle in degrees
    """
    image_center_x = camera_pixel_width / 2.0
    return math.degrees(math.atan(((x_offset - image_center_x) * 0.005246675486)))


def calculate_relative_pose(bbox_bounds, input_size, label_shape, FOCAL_LENGTH, SENSOR_SIZE, adjustment_factor):
    """
        Returns rel pose, to be used as a part of the CVObject

        Parameters:
            bbox_bounds: the detection object
            input_size: array wrt input size ([0] is width, [1] is height)
            label_shape: the label shape ([0] is width --> only this is accessed, [1] is height)
            FOCAL_LENGTH: a constant to pass in
            SENSOR_SIZE: a constant to pass in
            adjustment_factor: 1 if mono 2 if depthai
    """
    xmin, ymin, xmax, ymax = bbox_bounds

    bbox_width = (xmax - xmin) * input_size[0]
    bbox_center_x = (xmin + xmax) / 2 * input_size[0]
    bbox_center_y = (ymin + ymax) / 2 * input_size[1]
    meters_per_pixel = label_shape[0] / bbox_width
    dist_x = bbox_center_x - input_size[0] // 2
    dist_y = bbox_center_y - input_size[1] // 2

    y_meters = dist_x * meters_per_pixel * -1
    z_meters = dist_y * meters_per_pixel * -1

    # rospy.loginfo(bbox_width, bbox_center_x, bbox_center_y, )

    # isp_img_to_det_ratio = ISP_IMG_SHAPE[0] / model['input_size'][0]

    # print(bbox_width)
    x_meters = cam_dist_with_obj_width(bbox_width, label_shape[0], FOCAL_LENGTH, input_size, SENSOR_SIZE,
                                       adjustment_factor)

    return [x_meters, y_meters, z_meters]


def compute_bbox_dimensions(polygon):
    """
        Returns a CVObject messages, containing the following properties of the given Polygon:
            width, height, xmin, ymin, xmax, ymax as

        Args:
            polygon: Polygon object
    """

    # Ensure there are points in the polygon
    if len(polygon.points) < 4:
        raise ValueError("Polygon does not represent a bounding box with four points.")

    # Initialize min_x, max_x, min_y, and max_y with the coordinates of the first point
    min_x = polygon.points[0].x
    max_x = polygon.points[0].x
    min_y = polygon.points[0].y
    max_y = polygon.points[0].y

    # Iterate through all points to find the min and max x and y coordinates
    for point in polygon.points:
        if point.x < min_x:
            min_x = point.x
        if point.x > max_x:
            max_x = point.x
        if point.y < min_y:
            min_y = point.y
        if point.y > max_y:
            max_y = point.y

    # Compute the width and height
    width = max_x - min_x
    height = max_y - min_y

    # Create and populate message of CVObject type using polygon fields
    msg = CVObject()

    msg.width = width
    msg.height = height

    msg.xmin = min_x
    msg.xmax = max_x

    msg.ymin = min_y
    msg.ymax = max_y

    # TODO double check is coords field of CVObject centre?
    center = Point()
    center.x = (min_x + max_x) / 2
    center.y = (min_y + max_y) / 2

    msg.coords = center

    # TODO make this not 0 and actually what its supposed to be
    msg.yaw = 0

    return msg


def compute_center_distance(bbox_center_x, bbox_center_y, frame_width, frame_height, width_adjustment_constant=0,
                            height_adjustment_constant=0):

    '''
    Note that x, y is in the camera's reference frame
    '''

    # Compute the center of the frame
    frame_center_x = frame_width / 2
    frame_center_y = frame_height / 2

    # Compute the distances between the centers
    distance_x = bbox_center_x - frame_center_x + height_adjustment_constant
    distance_y = bbox_center_y - frame_center_y + width_adjustment_constant

    return distance_x, distance_y


class DetectionVisualizer:
    """
    Helper methods to visualize detections on an image feed. Adapted from class TextHelper:
    https://github.com/luxonis/depthai-experiments/blob/master/gen2-display-detections/utility.py
    """

    def __init__(self, classes, colors, show_class_name=True, show_confidence=True) -> None:
        self.text_type = cv2.FONT_HERSHEY_SIMPLEX
        self.line_type = cv2.LINE_AA

        # A list of classes of the model used for detection
        self.classes = classes
        self.colors = []
        for color in colors:
            self.colors.append(self.hex_to_rgb(color))
        self.show_class_name = show_class_name
        self.show_confidence = show_confidence

    # converts the hex string passed in by the args into a tuple representing the corresponding rgb color
    def hex_to_rgb(self, hex):
        return tuple(int(hex[i:i+2], 16) for i in (0, 2, 4))

    def putText(self, frame, text, coords, color):
        """Add text to frame, such as class label or confidence value."""

        (w, h), _ = cv2.getTextSize(text, self.text_type, 0.75, 2)
        # places the text labeling the class and/or confidence value of the bbox
        if coords[1]-h-10 > 0:
            # text is placed above the top left corner of the bbox by default
            new_coords = (coords[0], coords[1]-10)
            startpoint = (new_coords[0], new_coords[1]-h)
            endpoint = (new_coords[0] + w, new_coords[1]+10)
        else:
            # if there is not enough space above the top left corner of the bbox then
            # the text is placed right below the top left corner, within the bbox
            new_coords = (coords[0], coords[1]+h)
            startpoint = (new_coords[0],  new_coords[1]-h)
            endpoint = (new_coords[0] + w, new_coords[1])
        cv2.rectangle(frame, startpoint, endpoint, color, -1)
        cv2.putText(frame, text, new_coords, self.text_type, 0.75, (255, 255, 255), 2, self.line_type)

    def rectangle(self, frame, bbox, color):
        """Add a rectangle to frame, such as a bounding box."""
        x1, y1, x2, y2 = bbox
        # cv2.rectangle(frame, (x1, y1), (x2, y2), self.bg_color, 3)
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 3)

    def frame_norm(self, frame, bbox):
        """Normalize bbox locations between frame width/height."""
        norm_vals = np.full(len(bbox), frame.shape[0])
        norm_vals[::2] = frame.shape[1]
        return (np.clip(np.array(bbox), 0, 1) * norm_vals).astype(int)

    def visualize_detections(self, frame, detections):
        """ Returns frame with bounding boxes, classes, and labels of each detection overlaid."""
        frame_copy = frame.copy()

        for detection in detections:
            bbox = self.frame_norm(frame_copy, (detection.xmin, detection.ymin, detection.xmax, detection.ymax))
            # the code below specifies whether to display the bbox's class name and/or confidence value
            if self.show_class_name and self.show_confidence:
                self.putText(frame_copy, f"{self.classes[detection.label]} {int(detection.confidence * 100)}%",
                             (bbox[0], bbox[1]), self.colors[detection.label])
            elif self.show_class_name and not self.show_confidence:
                self.putText(frame_copy, self.classes[detection.label],
                             (bbox[0], bbox[1]), self.colors[detection.label])
            elif not self.show_class_name and self.show_confidence:
                self.putText(frame_copy, f"{int(detection.confidence * 100)}%",
                             (bbox[0], bbox[1]), self.colors[detection.label])

            self.rectangle(frame_copy, bbox, self.colors[detection.label])

        return frame_copy
