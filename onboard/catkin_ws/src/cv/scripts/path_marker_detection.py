#!/usr/bin/env python3

import rospy
import rostopic
import cv2

from custom_msgs.msg import PathMarker
from image_tools import ImageTools
from utils import DetectionVisualizer
import math
# from detecto.core import Model


class PathMarkerDetector:
    """This class computes and publishes predictions on a image stream."""
    LOWER_ORANGE = [5, 50, 50]
    UPPER_ORANGE = [33, 255, 255]
    PATH_MARKER_PUBLISH_TOPIC = ""

    # Load in models and other misc. setup work
    def __init__(self):
        rospy.init_node('path_marker', anonymous=True)

        self.image_tools = ImageTools()
        self.camera = "downwards"

        # The topic that the camera publishes its feed to
        self.camera_feed_topic = f'/camera/{self.camera}/compressed'
        self.publisher = rospy.Publisher(self.PATH_MARKER_PUBLISH_TOPIC, PathMarker, queue_size=10)

        self.detection_visualizer = DetectionVisualizer()

    def detect(self, img_msg):
        """Compute predictions on a raw image frame and publish results.

        This can be used as the camera topic subscriber callback.

        :param img_msg: ROS Image message to compute predictions on.
        """

        image = self.image_tools.convert_to_cv2(img_msg)
        height, width, _ = image.shape

        # convert image to HSV
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        orange_mask = cv2.inRange(hsv_image, self.LOWER_ORANGE, self.UPPER_ORANGE)

        # Apply Gaussian blurring
        blurred_mask = cv2.GaussianBlur(orange_mask, (5, 5), 0)

        # Find contours in the mask
        contours, _ = cv2.findContours(blurred_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Fit a line to the largest contour
        if len(contours) > 0:
            largest_contour = max(contours, key=cv2.contourArea)
            path_marker = cv2.fitEllipse(largest_contour)

            # Extract the center and orientation of the ellipse
            center_x = path_marker[0][0] / width
            center_y = path_marker[0][1] / height

            width, height = path_marker[1]
            orientation = - math.radians(path_marker[2])

            path_marker_msg = PathMarker()

            path_marker_msg.center.x = center_x
            path_marker_msg.center.y = center_y
            path_marker_msg.angle = orientation

            self.publisher.publish(path_marker_msg)

            self.detection_visualizer.slanted_rectangle(image, (center_x, center_y), (major_axis, minor_axis),
                                                        orientation, "386720")

    def run(self):
        """Initialize node and set up Subscriber to generate and publish predictions at every camera frame received."""
        TopicType, _, _ = rostopic.get_topic_class(self.camera_feed_topic)
        rospy.Subscriber(self.camera_feed_topic, TopicType, self.detect)

        # Keep node running until shut down
        rospy.spin()


if __name__ == '__main__':
    try:
        PathMarkerDetector().run()
    except rospy.ROSInterruptException:
        pass
