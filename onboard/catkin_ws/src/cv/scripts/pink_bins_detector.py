#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import math
from sklearn.cluster import DBSCAN
from sensor_msgs.msg import CompressedImage, Image
from custom_msgs.msg import CVObject
from std_msgs.msg import Float64
from cv_bridge import CvBridge, CvBridgeError
from custom_msgs.msg import RectInfo
from utils import compute_yaw

class PinkBinsDetector:

    MONO_CAM_IMG_SHAPE = (640, 480)  # Width, height in pixels

    def __init__(self):
        self.bridge = CvBridge()

        self.camera = rospy.get_param("~camera")
        rospy.init_node(f"{self.camera}_pink_bins_detector", anonymous=True)

        self.image_sub = rospy.Subscriber(f"/camera/usb/{self.camera}/compressed", CompressedImage, self.image_callback,
                                          queue_size=1)

        self.pink_bins_hsv_filtered_pub = rospy.Publisher(f"/cv/{self.camera}/pink_bins/hsv_filtered", Image, queue_size=10)
        self.pink_bins_dbscan_pub = rospy.Publisher(f"/cv/{self.camera}/pink_bins/dbscan", Image, queue_size=10)
        self.pink_bins_detections_pub = rospy.Publisher(f"/cv/{self.camera}/pink_bins/detections", Image, queue_size=10)
        self.pink_bins_bounding_box_pub = rospy.Publisher(f"/cv/{self.camera}/pink_bins/bounding_box", CVObject, queue_size=10)

    def image_callback(self, data):
        # Convert the compressed ROS image to OpenCV format
        np_arr = np.frombuffer(data.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Process the frame to find and publish information on the bin
        self.process_frame(frame)

    def process_frame(self, frame):
        # Convert frame to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define range for blue color and create mask
        lower_pink = np.array([125, 50, 150])
        upper_pink = np.array([170, 255, 255])
        mask = cv2.inRange(hsv, lower_pink, upper_pink)
        hsv_filtered_msg = self.bridge.cv2_to_imgmsg(mask, "mono8")
        self.pink_bins_hsv_filtered_pub.publish(hsv_filtered_msg)

        points = np.argwhere(mask > 0)
        dbscan_img = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

        db = DBSCAN(eps=2, min_samples=10).fit(points)
        labels = db.labels_

        unique_labels = set(labels)
        cluster_counts = {k: np.sum(labels == k) for k in unique_labels if k != -1}

        if cluster_counts == {}:
            frame_msg = self.bridge.cv2_to_imgmsg(frame, "rgb8")
            self.pink_bins_dbscan_pub.publish(hsv_filtered_msg)
            self.pink_bins_detections_pub.publish(frame_msg)
            return

        sorted_cluster_labels = sorted(cluster_counts, key=cluster_counts.get, reverse=True)
        colors = [(0, 0, 255), (0, 255, 255), (255, 0, 0)]
        final_x, final_y = 0, 0
        chosen_label_score = None
        for label, color in zip(sorted_cluster_labels[:3], colors):
            class_member_mask = (labels == label)
            max_clust_points = points[class_member_mask]

            for point in max_clust_points:
                dbscan_img[point[0], point[1]] = (0, 255, 0)

            center_x = np.mean(max_clust_points[:, 1])
            center_y = np.mean(max_clust_points[:, 0])

            if center_y > final_y:
                final_x = center_x
                final_y = center_y
                chosen_label_score = max_clust_points.shape[0]

        final_point_int = (int(final_x), int(final_y))

        cv2.circle(dbscan_img, final_point_int, 7, (0, 0, 255), -1)

        dbscan_msg = self.bridge.cv2_to_imgmsg(dbscan_img, "bgr8")
        self.pink_bins_dbscan_pub.publish(dbscan_msg)

        # Publish the frame with lowest point marked
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        cv2.circle(frame, final_point_int, 7, (255, 0, 0), -1)
        frame_msg = self.bridge.cv2_to_imgmsg(frame, "rgb8")
        self.pink_bins_detections_pub.publish(frame_msg)

        # Publish the bounding box of the bin
        final_x_normalized = final_x / self.MONO_CAM_IMG_SHAPE[0]
        cv_object = CVObject()
        cv_object.header.stamp = rospy.Time.now()
        cv_object.yaw = -compute_yaw(final_x_normalized, final_x_normalized, self.MONO_CAM_IMG_SHAPE[0])
        cv_object.score = chosen_label_score
        self.pink_bins_bounding_box_pub.publish(cv_object)


if __name__ == "__main__":
    detector = PinkBinsDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down Pink Bins Detector node")