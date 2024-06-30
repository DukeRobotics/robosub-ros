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

        path_to_reference_img = rr.get_filename('package://cv/assets/polyform-a0-buoy-contour.png', use_protocol=False)

        # Load the reference image in grayscale (assumes the image is already binary: white and black)
        self.reference_image = cv2.imread(path_to_reference_img, cv2.IMREAD_GRAYSCALE)

        # Compute the contours directly on the binary image
        self.ref_contours, _ = cv2.findContours(self.reference_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/usb_camera/compressed', CompressedImage, self.image_callback)
        self.bounding_box_pub = rospy.Publisher('/contour_matching/bounding_box', Polygon, queue_size=10)
        self.hsv_filtered_pub = rospy.Publisher('/contour_matching/hsv_filtered', Image, queue_size=10)
        self.contour_image_pub = rospy.Publisher('/contour_matching/contour_image', Image, queue_size=10)
        self.contour_image_with_bbox_pub = rospy.Publisher('/contour_matching/contour_image_with_bbox', Image, queue_size=10)

        self.last_n_bboxes = []
        self.n = 10  # Set the number of last bounding boxes to store

    def image_callback(self, data):
        try:
            # Convert the image from the compressed format to OpenCV format
            np_arr = np.frombuffer(data.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        except Exception as e:
            rospy.logerr("Failed to convert image: %s", e)
            return

        # Define the range for HSV filtering on the red buoy
        lower_red = np.array([0, 120, 70])
        upper_red = np.array([10, 255, 255])
        lower_red_upper = np.array([170, 120, 70])
        upper_red_upper = np.array([180, 255, 255])

        # Apply HSV filtering on the image
        mask1 = cv2.inRange(hsv_image, lower_red, upper_red)
        mask2 = cv2.inRange(hsv_image, lower_red_upper, upper_red_upper)
        red_hsv = cv2.bitwise_or(mask1, mask2)

        # Apply morphological operations to clean up the binary image
        kernel = np.ones((5, 5), np.uint8)
        red_hsv = cv2.morphologyEx(red_hsv, cv2.MORPH_CLOSE, kernel)

        # Publish the HSV filtered image
        hsv_filtered_msg = self.bridge.cv2_to_imgmsg(red_hsv, "mono8")
        self.hsv_filtered_pub.publish(hsv_filtered_msg)

        # Find contours in the image
        contours, _ = cv2.findContours(red_hsv, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        image_with_contours = image.copy()
        cv2.drawContours(image_with_contours, contours, -1, (255, 0, 0), 2)

        contour_image_msg = self.bridge.cv2_to_imgmsg(image_with_contours, "bgr8")
        self.contour_image_pub.publish(contour_image_msg)

        best_cnt = None
        best_match = float('inf')

        # Match contours with the reference image contours
        for cnt in contours:
            match = cv2.matchShapes(self.ref_contours[0], cnt, cv2.CONTOURS_MATCH_I1, 0.0)
            if match < best_match:
                best_match = match
                best_cnt = cnt

        if best_cnt is not None and best_match < 15:
            x, y, w, h = cv2.boundingRect(best_cnt)
            bbox = (x, y, w, h)
            self.last_n_bboxes.append(bbox)
            if len(self.last_n_bboxes) > self.n:
                self.last_n_bboxes.pop(0)

            filtered_bboxes = self.filter_outliers(self.last_n_bboxes)
            if filtered_bboxes:
                central_bbox = self.find_most_central_bbox(filtered_bboxes)
                self.publish_bbox(central_bbox, image)

    def filter_outliers(self, bboxes):
        if len(bboxes) <= 2:
            return bboxes

        centers = [(x + w / 2, y + h / 2) for x, y, w, h in bboxes]
        mean_center = np.mean(centers, axis=0)
        distances = [np.linalg.norm(np.array(center) - mean_center) for center in centers]
        std_distance = np.std(distances)
        filtered_bboxes = [bboxes[i] for i in range(len(bboxes)) if distances[i] <= mean_center[0] + 2 * std_distance]
        return filtered_bboxes

    def find_most_central_bbox(self, bboxes):
        centers = [(x + w / 2, y + h / 2) for x, y, w, h in bboxes]
        mean_center = np.mean(centers, axis=0)
        distances = [np.linalg.norm(np.array(center) - mean_center) for center in centers]
        central_bbox_index = np.argmin(distances)
        return bboxes[central_bbox_index]

    def publish_bbox(self, bbox, image):
        x, y, w, h = bbox
        bounding_box = Polygon()
        bounding_box.points = [Point32(x, y, 0), Point32(x + w, y, 0), Point32(x + w, y + h, 0), Point32(x, y + h, 0)]
        self.bounding_box_pub.publish(bounding_box)

        # Draw bounding box on the image
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # Convert the image with the bounding box to ROS Image message and publish
        image_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
        self.contour_image_with_bbox_pub.publish(image_msg)

if __name__ == '__main__':
    try:
        node = BuoyDetectorContourMatching()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
