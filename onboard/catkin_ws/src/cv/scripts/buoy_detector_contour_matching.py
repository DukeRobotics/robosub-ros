#!/usr/bin/env python

import rospy
import resource_retriever as rr
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image
from geometry_msgs.msg import Polygon, Point32

class BuoyDetectorContourMatching:
    def __init__(self):
        rospy.init_node('buoy_detector_contour_matching', anonymous=True)

        path_to_reference_img = rr.get_filename('package://cv/assets/polyform-a0-buoy.png', use_protocol=False)

        # Load the reference image and find its contours
        self.reference_image = cv2.imread(path_to_reference_img, cv2.IMREAD_COLOR)
        red_channel = self.reference_image[:, :, 2]  # Extract the red channel

        # # Define the range for thresholding on the red channel
        lower_red = 230  # Lower threshold for red values
        upper_red = 255  # Upper threshold for red values

        # Apply thresholding on the red channel
        _, thresh = cv2.threshold(red_channel, lower_red, 255, cv2.THRESH_BINARY)
        _, thresh_upper = cv2.threshold(red_channel, upper_red, 255, cv2.THRESH_BINARY_INV)
        red_thresh = cv2.bitwise_and(thresh, thresh_upper)

        # _, self.ref_thresh = cv2.threshold(self.reference_image, 190, 255, cv2.THRESH_BINARY_INV)
        self.ref_contours, _ = cv2.findContours(red_thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/usb_camera/compressed', CompressedImage, self.image_callback)
        self.bounding_box_pub = rospy.Publisher('/contour_matching/bounding_box', Polygon, queue_size=10)
        self.image_pub = rospy.Publisher('/contour_matching/image_with_box', Image, queue_size=10)

    def image_callback(self, data):
        try:
            # Convert the image from the compressed format to OpenCV format
            np_arr = np.frombuffer(data.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            red_channel = image[:, :, 2]  # Extract the red channel
        except Exception as e:
            rospy.logerr("Failed to convert image: %s", e)
            return

        # Define the range for thresholding on the red channel
        lower_red = 220  # Lower threshold for red values
        upper_red = 255  # Upper threshold for red values

        # Apply thresholding on the red channel
        _, thresh = cv2.threshold(red_channel, lower_red, 255, cv2.THRESH_BINARY)
        _, thresh_upper = cv2.threshold(red_channel, upper_red, 255, cv2.THRESH_BINARY_INV)
        red_thresh = cv2.bitwise_and(thresh, thresh_upper)

        # Apply morphological operations to clean up the binary image
        kernel = np.ones((5, 5), np.uint8)
        red_thresh = cv2.morphologyEx(red_thresh, cv2.MORPH_CLOSE, kernel)

        # Find contours in the image
        contours, _ = cv2.findContours(red_thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Filter contours by area
        min_area = 500  # Adjust as necessary
        max_area = 5000  # Adjust as necessary
        filtered_contours = [cnt for cnt in contours if min_area < cv2.contourArea(cnt) < max_area]

        image_with_contours = image.copy()
        cv2.drawContours(image_with_contours, filtered_contours, -1, (255, 0, 0), 2)

        red_thresh = cv2.cvtColor(red_thresh, cv2.COLOR_GRAY2BGR)
        image_msg = self.bridge.cv2_to_imgmsg(image_with_contours, "bgr8")
        self.image_pub.publish(image_msg)

        match_list = []

        best_cnt = None
        best_match = float('inf')

        # Match contours with the reference image contours
        for cnt in filtered_contours:
            match = cv2.matchShapes(self.ref_contours[0], cnt, cv2.CONTOURS_MATCH_I1, 0.0)
            match_list.append(match)
            if match < best_match:
                best_match = match
                best_cnt = cnt

        x, y, w, h = cv2.boundingRect(best_cnt)
        bounding_box = Polygon()
        bounding_box.points = [Point32(x, y, 0), Point32(x + w, y, 0), Point32(x + w, y + h, 0), Point32(x, y + h, 0)]
        self.bounding_box_pub.publish(bounding_box)

        # Draw bounding box on the image
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # Convert the image with the bounding box to ROS Image message and publish
        # image_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
        # self.image_pub.publish(image_msg)

        match_list.sort()
        formatted_match_list = ['%.5f' % elem for elem in match_list[:5]]
        rospy.loginfo(formatted_match_list)


if __name__ == '__main__':
    try:
        node = BuoyDetectorContourMatching()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass