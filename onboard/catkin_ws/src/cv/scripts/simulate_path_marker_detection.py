#!/usr/bin/env python3

import cv2
import os
import math
from utils import visualize_path_marker_detection
import rospy
import resource_retriever as rr
from sensor_msgs.msg import CompressedImage
from custom_msgs.msg import PathMarker
from image_tools import ImageTools
import rostopic


class SimulatePathMarkerDetection:
    """
    This class is used to test a DepthAI neural network (blob file) with a simulated image feed, either a still image
    or an Image topic. This class takes the images, transfers them from the host (local computer) to the camera,
    and retrieves the output of the neural network.
    """
    def __init__(self):
        rospy.init_node('simulate_path_maker_detection', anonymous=True)

        self.feed_path = rospy.get_param("~feed_path")
        self.latest_img = None

        # No feed path is passed in -- throw an expcetion
        if self.feed_path == "":
            rospy.logerr("No feed path variable given")
            rospy.spin()

        self.image_tools = ImageTools()

        # Setup detection publishers
        self.publishing_topic = rospy.get_param("~publishing_topic")
        self.detection_publisher = rospy.Publisher(self.publishing_topic, PathMarker, queue_size=10)
        self.visualized_detection_publisher = rospy.Publisher(f'{self.publishing_topic}_visualized/compressed',
                                                              CompressedImage,
                                                              queue_size=10)

    def detect(self, image):
        height, width, _ = image.shape

        # Convert image to HSV
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

            orientation = - math.radians(path_marker[2])

            path_marker_msg = PathMarker()

            path_marker_msg.center.x = center_x
            path_marker_msg.center.y = center_y
            path_marker_msg.angle = orientation

            self.detection_publisher.publish(path_marker_msg)

            return {"center": (center_x, center_y), "dimensions": path_marker[1], "orientation": orientation}

    def _load_image_from_feed_path(self):
        """ Load a still image from the feed path """
        image_path = rr.get_filename(f"package://cv/assets/{self.feed_path}", use_protocol=False)
        image = cv2.imread(image_path, cv2.IMREAD_COLOR)
        return image

    def _feed_is_still_image(self):
        """ Check if the feed_path is to a still image

        Returns:
            bool: Whether the feed_path points to a still image
        """
        try:
            img = self._load_image_from_feed_path()
            return not (img is None or img.size == 0)
        except Exception:
            return False

    def _update_latest_img(self, img_msg):
        """ Send an image to the device for detection

        Args:
            img_msg (sensor_msgs.msg.CompressedImage): Image to send to the device
        """
        # Format a cv2 image to be sent to the device
        self.latest_img = self.image_tools.convert_to_cv2(img_msg)

    def _publish_visualized_detections(self, frame, detection):
        """ Publish the detection results visualized as an Image message

        Args:
            detection_results (dict): Output from detect()
        """
        visualized_detection_results = visualize_path_marker_detection(frame, detection)
        visualized_detection_results_msg = self.image_tools.convert_to_ros_compressed_msg(visualized_detection_results)

        self.visualized_detection_publisher.publish(visualized_detection_results_msg)

    def _run_detection_on_image_topic(self):
        """ Run and publish detections on the provided topic

        Args:
            device (depthai.Device): DepthAI device being used
        """
        TopicType, _, _ = rostopic.get_topic_class(self.feed_path)
        rospy.Subscriber(self.feed_path, TopicType, self._update_latest_img)

        while not rospy.is_shutdown():
            current_image = self.latest_img
            detection = self.detect(current_image)
            self._publish_visualized_detections(current_image, detection)

    def _run_detection_on_single_image(self, img):
        """ Run detection on the single image provided

        Args:
            device (depthai.Device): DepthAI device being used
            img (ndarray): Image to run detection on
        """
        latest_img = self.image_tools.convert_to_cv2(img)
        detection = self.detect(latest_img)
        visualized_detection_results = visualize_path_marker_detection(latest_img, detection)
        self._save_visualized_detection_results(visualized_detection_results)

    def _save_visualized_detection_results(self, visualized_detection_results):
        """ Save the visualized detection results as a jpg file

        Args:
            visualized_detection_results (ndarray): Image to save
        """
        detection_results_filename = f'{os.path.splitext(self.feed_path)[0]}_detection_results.jpg'
        detection_results_filepath = rr.get_filename(f"package://cv/assets/{detection_results_filename}",
                                                     use_protocol=False)
        cv2.imwrite(detection_results_filepath, visualized_detection_results)

    def run(self):
        """ Run detection on the latest img message """
        if self._feed_is_still_image():
            rospy.loginfo(f'Running detection on still image provided: {self.feed_path}')
            img = self._load_image_from_feed_path()
            self._run_detection_on_single_image(img)
        else:
            rospy.loginfo(f'Running detection on topic provided: {self.feed_path}')
            self._run_detection_on_image_topic()


if __name__ == '__main__':
    try:
        SimulatePathMarkerDetection().run()
    except rospy.ROSInterruptException:
        pass
