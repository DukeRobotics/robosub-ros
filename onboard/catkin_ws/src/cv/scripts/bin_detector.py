#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float64
from cv_bridge import CvBridge, CvBridgeError
from custom_msgs.msg import RectInfo


"""
TODO: make these changes in cv.py in task_planning/interface
    - z position of red and blue bins (this should be NEGATIVE)
    - Write callback functions for subscribers
    - Calculate bin center position from red and blue
        - x, y, z of bin center should all be average of red and blue bin's x, y, and z

    self.cv_data["bin_red"] = {
        "x": ...,
        "y": ...,
        "z": ...,
        "distance_x": ...,
        "distance_y":  ...,
        "fully_in_frame": ...,
        "time": rospy.Time.now()
    }
"""


class BinDetector:
    BIN_WIDTH = 0  # TODO: get this

    MONO_CAM_IMG_SHAPE = (640, 480)  # Width, height in pixels
    MONO_CAM_SENSOR_SIZE = (3.054, 1.718)  # Width, height in mm
    MONO_CAM_FOCAL_LENGTH = 2.65  # Focal length in mm

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/usb_camera/compressed", CompressedImage, self.image_callback)
        self.blue_hsv = rospy.Publisher("/cv/front_usb/blue_bin/contour_image", Image, queue_size=10)
        self.blue_bbox = rospy.Publisher("/cv/front_usb/blue_bin/bounding_box", RectInfo, queue_size=10)
        self.red_hsv = rospy.Publisher("/cv/front_usb/red_bin/contour_image", Image, queue_size=10)
        self.red_bbox = rospy.Publisher("/cv/front_usb/red_bin/bounding_box", RectInfo, queue_size=10)
        self.both_hsv = rospy.Publisher("/cv/front_usb/both_bin/contour_image", Image, queue_size=10)
        self.both_bbox = rospy.Publisher("/cv/front_usb/both_bin/bounding_box", RectInfo, queue_size=10)

    def image_callback(self, data):
        try:
            # Convert the compressed ROS image to OpenCV format
            np_arr = np.frombuffer(data.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            # Process the frame to find the angle of the blue rectangle and draw the rectangle
            boxes = self.process_frame(frame)
            rospy.loginfo(boxes)
            if boxes["red"]:
                contour_image_msg = self.bridge.cv2_to_imgmsg(boxes["red"][3], "bgr8")
                self.red_hsv.publish(contour_image_msg)
                self.red_bbox.publish(boxes["red"][2])
            if boxes["blue"]:
                contour_image_msg = self.bridge.cv2_to_imgmsg(boxes["blue"][3], "bgr8")
                self.blue_hsv.publish(contour_image_msg)
                self.blue_bbox.publish(boxes["blue"][2])
            if boxes["both"]:
                contour_image_msg = self.bridge.cv2_to_imgmsg(boxes["blue"][3], "bgr8")
                self.both_hsv.publish(contour_image_msg)
                self.both_bbox.publish(boxes["both"][2])
        except CvBridgeError as e:
            rospy.logerr(f"Could not convert image: {e}")

    def process_frame(self, frame):
        # Convert frame to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define range for blue color and create mask
        lower_blue = np.array([110, 230, 180])
        upper_blue = np.array([125, 260, 215])
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

        lower_red = np.array([0, 191, 191])
        upper_red = np.array([4, 255, 255])
        mask_red = cv2.inRange(hsv, lower_red, upper_red)

        # Find contours in the mask
        contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        contours_red, _ = cv2.findContours(mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        res = {"blue": None, "red": None, "both": None}

        if contours_blue:
            res["blue"] = self.get_rect_info(frame.copy(), contours_blue)
        if contours_red:
            res["red"] = self.get_rect_info(frame.copy(), contours_red)
        if contours_blue and contours_red:
            res["both"] = self.get_rect_info(frame.copy(), (contours_blue+contours_red))
        return res

    def get_rect_info(self,frame, contours):
        # Combine all contours to form the large rectangle
        all_points = np.vstack(contours)

        # Get the minimum area rectangle that encloses the combined contour
        rect = cv2.minAreaRect(all_points)

        # Draw the rectangle on the frame
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        cv2.drawContours(frame, [box], 0, (0, 0, 255), 3)

        # Sort the points based on their x-coordinates to identify left and right sides
        box = sorted(box, key=lambda pt: pt[0])

        # Identify left and right side points
        left_pts = box[:2]
        right_pts = box[2:]

        # Determine which point is higher on the left side
        left_top = min(left_pts, key=lambda pt: pt[1])

        # Determine which point is higher on the right side
        right_top = min(right_pts, key=lambda pt: pt[1])

        angle = rect[-1]

        # Compare the y-coordinates
        if right_top[1] < left_top[1]:
            # Right side is higher than left side
            angle = rect[-1] - 90

        if angle == -90 or angle == 90:
            angle = 0

        # Calculate the center of the rectangle
        rect_center = rect[0]

        # Calculate the center of the frame
        frame_center = (frame.shape[1] / 2, frame.shape[0] / 2)

        # Calculate the vertical distance between the two centers
        vertical_distance = rect_center[1] - frame_center[1]

        # The distance is positive if the rectangle is below the centerline, negative if above
        distance = -vertical_distance

        # Create RectInfo message
        rect_info = RectInfo()
        rect_info.center_x = rect_center[0]
        rect_info.center_y = rect_center[1]
        rect_info.width = rect[1][0]
        rect_info.height = rect[1][1]
        rect_info.angle = angle
        return angle, distance, rect_info, frame

if __name__ == "__main__":
    rospy.init_node('bin_detector', anonymous=True)
    detector = BinDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down Bin Detector node")