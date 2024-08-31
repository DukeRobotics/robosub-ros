#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import math
from sensor_msgs.msg import CompressedImage, Image
from custom_msgs.msg import CVObject
from geometry_msgs.msg import Point
from cv_bridge import CvBridge

from utils import compute_center_distance


class PathMarkerDetector:
    MONO_CAM_IMG_SHAPE = (640, 480)  # Width, height in pixels

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/usb/bottom/compressed", CompressedImage, self.image_callback)

        # define information published for path marker
        self.path_marker_hsv_filtered_pub = rospy.Publisher("/cv/bottom/path_marker/hsv_filtered", Image, queue_size=10)
        self.path_marker_contour_image_pub = rospy.Publisher("/cv/bottom/path_marker/contour_image", Image,
                                                             queue_size=10)
        self.path_marker_bounding_box_pub = rospy.Publisher("/cv/bottom/path_marker/bounding_box", CVObject,
                                                            queue_size=10)
        self.path_marker_distance_pub = rospy.Publisher("/cv/bottom/path_marker/distance", Point, queue_size=10)

    def image_callback(self, data):
        # Convert the compressed ROS image to OpenCV format
        np_arr = np.frombuffer(data.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        frame = frame[:, :-20]

        # Process the frame to find and publish information on the bin
        self.process_frame(frame)

    def process_frame(self, frame):
        # Convert frame to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define range for blue color and create mask
        lower_orange = np.array([0, 130, 100])
        upper_orange = np.array([20, 255, 255])
        # lower_orange = np.array([2, 80, 100])
        # upper_orange = np.array([33, 255, 255])
        mask = cv2.inRange(hsv, lower_orange, upper_orange)

        hsv_filtered_msg = self.bridge.cv2_to_imgmsg(mask, "mono8")
        self.path_marker_hsv_filtered_pub.publish(hsv_filtered_msg)

        # Apply morphological operations to clean up the binary image
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = [contour for contour in contours if len(contour) > 5 and cv2.contourArea(contour) > 500]

        # Fit a line to the largest contour
        if len(contours) > 0:
            largest_contour = max(contours, key=cv2.contourArea)
            center, dimensions, orientation = cv2.fitEllipse(largest_contour)

            # NOTE: CLOCKWISE yaw from 12 o'clock
            orientation_in_radians = math.radians(orientation)

            # create CVObject message and populate relevant attributes
            bounding_box = CVObject()

            bounding_box.header.stamp = rospy.Time.now()

            orientation_in_radians = math.pi / 2 - orientation_in_radians

            bounding_box.yaw = orientation_in_radians

            bounding_box.width = int(dimensions[0])
            bounding_box.height = int(dimensions[1])

            # Compute distance between center of bounding box and center of image
            # Here, image x is robot's y, and image y is robot's z
            dist_x, dist_y = compute_center_distance(center[0], center[1], *self.MONO_CAM_IMG_SHAPE)

            # get distances into publishable format
            # also fix axis from camera's POV
            dist_point = Point()
            dist_point.x = dist_x
            dist_point.y = -dist_y

            # publish dist, bbox
            self.path_marker_distance_pub.publish(dist_point)
            self.path_marker_bounding_box_pub.publish(bounding_box)

            visualized_frame = self.visualize_path_marker_detection(frame, center, bounding_box,
                                                                    math.pi / 2 - orientation_in_radians)
            cv2.circle(visualized_frame, (int(center[0]), int(center[1])), 5, (0, 0, 255), -1)
            self.path_marker_contour_image_pub.publish(self.bridge.cv2_to_imgmsg(visualized_frame))

    def visualize_path_marker_detection(self, frame, center, bounding_box, orientation):
        """Returns frame with bounding boxes of the detection."""
        frame_copy = frame.copy()

        center_x, center_y = center
        width, height = bounding_box.width, bounding_box.height
        orientation = orientation if orientation > 0 else orientation + math.pi

        # Calculate the four corners of the rectangle
        angle_cos = math.cos(orientation)
        angle_sin = math.sin(orientation)
        half_width = width / 2
        half_height = height / 2

        x1 = int(center_x - half_width * angle_cos + half_height * angle_sin)
        y1 = int(center_y - half_width * angle_sin - half_height * angle_cos)
        x2 = int(center_x + half_width * angle_cos + half_height * angle_sin)
        y2 = int(center_y + half_width * angle_sin - half_height * angle_cos)
        x3 = int(center_x + half_width * angle_cos - half_height * angle_sin)
        y3 = int(center_y + half_width * angle_sin + half_height * angle_cos)
        x4 = int(center_x - half_width * angle_cos - half_height * angle_sin)
        y4 = int(center_y - half_width * angle_sin + half_height * angle_cos)

        # Draw the rotated rectangle
        cv2.line(frame_copy, (x1, y1), (x2, y2), (0, 0, 255), 3)
        cv2.line(frame_copy, (x2, y2), (x3, y3), (0, 0, 255), 3)
        cv2.line(frame_copy, (x3, y3), (x4, y4), (0, 0, 255), 3)
        cv2.line(frame_copy, (x4, y4), (x1, y1), (0, 0, 255), 3)

        return frame_copy


if __name__ == "__main__":
    rospy.init_node('path_marker_detector', anonymous=True)
    detector = PathMarkerDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down Path Marker Detector node")
