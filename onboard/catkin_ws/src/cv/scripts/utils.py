#!/usr/bin/env python3

import numpy as np
import cv2


class DetectionVisualizer:
    """
    Helper methods to visualize detections on an image feed. Adapted from class TextHelper:
    https://github.com/luxonis/depthai-experiments/blob/master/gen2-display-detections/utility.py
    """

    def __init__(self, classes, colors, showClassName = True, showConfidence = True) -> None:

        # The color to outline text & bounding boxes in
        #self.bg_color = (0, 0, 0)

        # The color of the text & bounding boxes
        #self.color = (255, 255, 255)


        self.text_type = cv2.FONT_HERSHEY_SIMPLEX
        self.line_type = cv2.LINE_AA

        # A list of classes of the model used for detection
        self.classes = classes
        self.colors = []
        for color in colors:
            self.colors.append(self.hex_to_rgb(color))
        self.showClassName = showClassName
        self.showConfidence = showConfidence

    def hex_to_rgb(self, hex):
        return tuple(int(hex[i:i+2], 16) for i in (0, 2, 4))

    def putText(self, frame, text, coords, color):
        """Add text to frame, such as class label or confidence value."""
        #cv2.putText(frame, text, coords, self.text_type, 0.75, self.bg_color, 3, self.line_type)
        
        (w, h), _ = cv2.getTextSize(text, self.text_type, 0.75, 2)
        if coords[1]-h-10 > 0:
            newCoords = (coords[0], coords[1]-10)
            startpoint = (newCoords[0], newCoords[1]-h)
            endpoint = (newCoords[0] + w, newCoords[1]+10)
        else:
            newCoords = (coords[0], coords[1]+h)
            startpoint = (newCoords[0],  newCoords[1]-h)
            endpoint = (newCoords[0] + w, newCoords[1])
        cv2.rectangle(frame, startpoint, endpoint, color, -1)
        cv2.putText(frame, text, newCoords, self.text_type, 0.75, (255,255,255), 2, self.line_type)
        
    def rectangle(self, frame, bbox, color):
        """Add a rectangle to frame, such as a bounding box."""
        x1, y1, x2, y2 = bbox
        #cv2.rectangle(frame, (x1, y1), (x2, y2), self.bg_color, 3)
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
            if self.showClassName and self.showConfidence:
                self.putText(frame_copy, f"{self.classes[detection.label]} {int(detection.confidence * 100)}%", (bbox[0], bbox[1]), self.colors[detection.label])
            elif self.showClassName and not self.showConfidence:
                self.putText(frame_copy, self.classes[detection.label], (bbox[0], bbox[1]), self.colors[detection.label])
            elif not self.showClassName and self.showConfidence:
                self.putText(frame_copy, f"{int(detection.confidence * 100)}%", (bbox[0], bbox[1]), self.colors[detection.label])
            
            self.rectangle(frame_copy, bbox, self.colors[detection.label])

        return frame_copy
