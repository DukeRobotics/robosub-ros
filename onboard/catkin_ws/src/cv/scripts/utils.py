#!/usr/bin/env python3

import numpy as np
import cv2
import os
import math


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


# converts the hex string passed in by the args into a tuple representing the corresponding rgb color
def hex_to_rgb(hex):
    return tuple(int(hex[i:i+2], 16) for i in (0, 2, 4))


def visualize_path_marker_detection(frame, detection, color="386720"):
    """Returns frame with bounding boxes of the detection."""
    frame_copy = frame.copy()

    center_x, center_y = detection["center"]
    width, height = detection["dimensions"]
    orientation = detection["orientation"]

    color = hex_to_rgb(color)

    # Calculate the four corners of the rectangle
    angle_cos = math.cos(orientation)
    angle_sin = math.sin(orientation)
    half_width = width / 2
    half_height = height / 2

    x1 = int(center_x - half_width * angle_cos - half_height * angle_sin)
    y1 = int(center_y + half_width * angle_sin - half_height * angle_cos)
    x2 = int(center_x + half_width * angle_cos - half_height * angle_sin)
    y2 = int(center_y - half_width * angle_sin - half_height * angle_cos)
    x3 = int(2 * center_x - x1)
    y3 = int(2 * center_y - y1)
    x4 = int(2 * center_x - x2)
    y4 = int(2 * center_y - y2)

    # Draw the rotated rectangle
    cv2.line(frame_copy, (x1, y1), (x2, y2), color, 3)
    cv2.line(frame_copy, (x2, y2), (x3, y3), color, 3)
    cv2.line(frame_copy, (x3, y3), (x4, y4), color, 3)
    cv2.line(frame_copy, (x4, y4), (x1, y1), color, 3)

    return frame_copy


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
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 3)

    def frame_norm(self, frame, bbox):
        """Normalize bbox locations between frame width/height."""
        norm_vals = np.full(len(bbox), frame.shape[0])
        norm_vals[::2] = frame.shape[1]
        return (np.clip(np.array(bbox), 0, 1) * norm_vals).astype(int)

    def visualize_detections(self, frame, detections):
        """Returns frame with bounding boxes, classes, and labels of each detection overlaid."""
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
