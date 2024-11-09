#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
from custom_msgs.msg import CVObject
from geometry_msgs.msg import Point
from utils import compute_yaw, calculate_relative_pose, compute_center_distance


class BinDetector:
    BIN_WIDTH = 0.3048  # width of one square of the bin, in m

    MONO_CAM_IMG_SHAPE = (640, 480)  # Width, height in pixels
    MONO_CAM_SENSOR_SIZE = (3.054, 1.718)  # Width, height in mm
    MONO_CAM_FOCAL_LENGTH = 2.65  # Focal length in mm

    def __init__(self):
        self.bridge = CvBridge()
        # subscribe to image topic to get images
        self.image_sub = rospy.Subscriber("/camera/usb/bottom/compressed", CompressedImage, self.image_callback)

        # blue bin publiishers
        self.blue_bin_hsv_filtered_pub = rospy.Publisher("/cv/bottom/bin_blue/hsv_filtered", Image, queue_size=10)
        self.blue_bin_contour_image_pub = rospy.Publisher("/cv/bottom/bin_blue/contour_image", Image, queue_size=10)
        self.blue_bin_bounding_box_pub = rospy.Publisher("/cv/bottom/bin_blue/bounding_box", CVObject, queue_size=10)
        self.blue_bin_distance_pub = rospy.Publisher("/cv/bottom/bin_blue/distance", Point, queue_size=10)

        # red bin publishers
        self.red_bin_hsv_filtered_pub = rospy.Publisher("/cv/bottom/bin_red/hsv_filtered", Image, queue_size=10)
        self.red_bin_contour_image_pub = rospy.Publisher("/cv/bottom/bin_red/contour_image", Image, queue_size=10)
        self.red_bin_bounding_box_pub = rospy.Publisher("/cv/bottom/bin_red/bounding_box", CVObject, queue_size=10)
        self.red_bin_distance_pub = rospy.Publisher("/cv/bottom/bin_red/distance", Point, queue_size=10)

        # centre bin publsihers (NOTE we don't actually publish to these heheheha)
        self.bin_center_hsv_filtered_pub = rospy.Publisher("/cv/bottom/bin_center/hsv_filtered", Image, queue_size=10)
        self.bin_center_contour_image_pub = rospy.Publisher("/cv/bottom/bin_center/contour_image", Image, queue_size=10)
        self.bin_center_bounding_box_pub = rospy.Publisher("/cv/bottom/bin_center/bounding_box", CVObject,
                                                           queue_size=10)
        self.bin_center_distance_pub = rospy.Publisher("/cv/bottom/bin_center/distance", Point, queue_size=10)

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
        lower_blue = np.array([90, 150, 50])
        upper_blue = np.array([125, 255, 255])
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

        blue_hsv_filtered_msg = self.bridge.cv2_to_imgmsg(mask_blue, "mono8")
        self.blue_bin_hsv_filtered_pub.publish(blue_hsv_filtered_msg)

        # Define the range for HSV filtering on the red bin
        lower_red_low = np.array([0, 110, 150])
        upper_red_low = np.array([12, 255, 255])
        lower_red_high = np.array([170, 50, 85])
        upper_red_high = np.array([179, 255, 255])

        # Apply HSV filtering on the image
        mask_red1 = cv2.inRange(hsv, lower_red_low, upper_red_low)
        mask_red2 = cv2.inRange(hsv, lower_red_high, upper_red_high)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)

        # convert cv2 image to img message, and publish the image
        red_hsv_filtered_msg = self.bridge.cv2_to_imgmsg(mask_red, "mono8")
        self.red_bin_hsv_filtered_pub.publish(red_hsv_filtered_msg)

        # Apply morphological operations to clean up the binary image
        kernel = np.ones((5, 5), np.uint8)
        mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, kernel)

        # Apply morphological operations to clean up the binary image
        mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
        mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)

        # Find contours in the mask
        contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours_red, _ = cv2.findContours(mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if contours_blue:
            # takes largest contour
            contours_blue = sorted(contours_blue, key=cv2.contourArea, reverse=True)
            contours_blue = contours_blue[0]

            # only processes if area (in pixels) if selected contour >500
            # publishes bbox, image, distance
            bbox, image, dist = self.process_contours(frame.copy(), contours_blue)
            if bbox and image and dist and cv2.contourArea(contours_blue) > 500:
                self.blue_bin_contour_image_pub.publish(image)
                self.blue_bin_bounding_box_pub.publish(bbox)
                self.blue_bin_distance_pub.publish(dist)

        if contours_red:
            # takes largest contour
            contours_red = sorted(contours_red, key=cv2.contourArea, reverse=True)
            contours_red = contours_red[0]

            # only processes if area (in pixels) if selected contour >500
            # publishes bbox, image, distance
            bbox, image, dist = self.process_contours(frame.copy(), contours_red)
            if bbox and image and dist and cv2.contourArea(contours_red) > 500:
                self.red_bin_contour_image_pub.publish(image)
                self.red_bin_bounding_box_pub.publish(bbox)
                self.red_bin_distance_pub.publish(dist)

    def process_contours(self, frame, contours):
        # Combine all contours to form the large rectangle
        all_points = np.vstack(contours)

        # Get the minimum area rectangle that encloses the combined contour
        rect = cv2.minAreaRect(all_points)

        # Draw the rectangle on the frame
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        cv2.drawContours(frame, [box], 0, (0, 0, 255), 3)

        # Calculate the center of the rectangle
        rect_center = rect[0]

        x, y, w, h = (rect_center[0], rect_center[1], rect[1][0], rect[1][1])

        # edge cases integer rounding idk something
        if (w == 0):
            return None, None, None

        # get dimensions, attributes of relevant shapes
        meters_per_pixel = self.BIN_WIDTH / w

        # create CVObject message, and populate relavent attributes
        bounding_box = CVObject()

        bounding_box.header.stamp.secs = rospy.Time.now().secs
        bounding_box.header.stamp.nsecs = rospy.Time.now().nsecs

        # gets dimns that CVObject wants for our rectangle
        bounding_box.xmin = (x) * meters_per_pixel
        bounding_box.ymin = (y) * meters_per_pixel
        bounding_box.xmax = (x + w) * meters_per_pixel
        bounding_box.ymax = (y + h) * meters_per_pixel

        bounding_box.yaw = compute_yaw(x, x + w, self.MONO_CAM_SENSOR_SIZE[0])  # width of camera in in mm

        bounding_box.width = int(w)
        bounding_box.height = int(h)

        # Compute distance between center of bounding box and center of image
        # Here, image x is robot's y, and image y is robot's z
        dist_x, dist_y = compute_center_distance(x, y, *self.MONO_CAM_IMG_SHAPE, height_adjustment_constant=15,
                                                 width_adjustment_constant=10)

        # Compute distance between center of bounding box and center of image in meters
        # dist_x_meters = dist_x * meters_per_pixel
        # dist_y_meters = dist_y * meters_per_pixel
        # dist_z_meters = -1 * self.mono_cam_dist_with_obj_width(w, self.BIN_WIDTH)

        # create Point message type, and populate x and y distances
        dist_point = Point()
        # dist_point.z = dist_z_meters
        dist_point.x = dist_x
        dist_point.y = -dist_y

        bbox_bounds = (x / self.MONO_CAM_IMG_SHAPE[0], y / self.MONO_CAM_IMG_SHAPE[1], (x+w) /
                       self.MONO_CAM_IMG_SHAPE[0], (y+h) / self.MONO_CAM_IMG_SHAPE[1])

        # Point coords represents the 3D position of the object represented by the bounding box relative to the robot
        coords_list = calculate_relative_pose(bbox_bounds,
                                              self.MONO_CAM_IMG_SHAPE,
                                              (self.BIN_WIDTH, 0),
                                              self.MONO_CAM_FOCAL_LENGTH,
                                              self.MONO_CAM_SENSOR_SIZE, 1)
        bounding_box.coords.x, bounding_box.coords.y, bounding_box.coords.z = coords_list

        # Convert the image with the bounding box to ROS Image message and publish
        cv2.circle(frame, (int(x), int(y)), 5, (0, 0, 255), -1)
        image_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        return bounding_box, image_msg, dist_point

    def mono_cam_dist_with_obj_width(self, width_pixels, width_meters):
        return (self.MONO_CAM_FOCAL_LENGTH * width_meters * self.MONO_CAM_IMG_SHAPE[0]) \
            / (width_pixels * self.MONO_CAM_SENSOR_SIZE[0])


if __name__ == "__main__":
    rospy.init_node('bin_detector', anonymous=True)
    detector = BinDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down Bin Detector node")
