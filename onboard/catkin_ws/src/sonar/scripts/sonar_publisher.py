#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sonar import Sonar
from custom_msgs.msg import sweepResult, sweepGoal
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from sonar_utils import degrees_to_centered_gradians
from sonar_image_processing import build_sonar_image


class SonarPublisher:

    SONAR_REQUEST_TOPIC = 'sonar/request'
    SONAR_RESPONSE_TOPIC = 'sonar/cv/response'
    SONAR_IMAGE_TOPIC = 'sonar/image/compressed'

    NODE_NAME = "sonar_pub"

    SONAR_DEFAULT_RANGE = 10

    CONSTANT_SWEEP_START = -180
    CONSTANT_SWEEP_END = 180

    def __init__(self):
        rospy.init_node(self.NODE_NAME)
        self.stream = rospy.get_param('~stream')
        self.polar = rospy.get_param('~polar')
        self.debug = rospy.get_param('~debug')
        self.sonar = Sonar(10)
        self.cv_bridge = CvBridge()
        self._pub_request = rospy.Publisher(self.SONAR_RESPONSE_TOPIC,
                                            sweepResult, queue_size=0)
        if self.stream:
            self.sonar_image_publisher = rospy.Publisher(self.SONAR_IMAGE_TOPIC,
                                                        CompressedImage)
    def on_request(self, request):
        if (request.distance_of_scan == -1):
            return
        self.sonar.set_new_range(request.distance_of_scan)

        left_gradians = degrees_to_centered_gradians(request.start_angle)
        right_gradians = degrees_to_centered_gradians(request.end_angle)

        sonar_x, sonar_y, scanned_image = self.sonar.get_xy_of_object_in_sweep(left_gradians,
                                                                              right_gradians)
        sonar_xy_result = (sonar_x, sonar_y)

        if self.stream:
            sonar_image = build_sonar_image(scanned_image)
            
            if self.polar:
                img = np.pad(sonar_image.astype(np.uint8), ((80, 80),(0, 0)), 'constant')
                greyscale_image = cv2.cvtColor(img.astype(np.uint8), cv2.COLOR_GRAY2BGR)
                polar_img = cv2.linearPolar(greyscale_image, (175, 175), 175.0, cv2.WARP_INVERSE_MAP)
                sonar_image = polar_img[0:350, 0:350]
            
            sonar_image = cv2.applyColorMap(sonar_image, cv2.COLORMAP_VIRIDIS)
            compressed_image = self.cv_bridge.cv2_to_compressed_imgmsg(sonar_image)
            self.sonar_image_publisher.publish(compressed_image)

        response = sweepResult()
        response.x_pos = sonar_xy_result[0]
        response.y_pos = sonar_xy_result[1]
        self._pub_request.publish(response)

    def constant_sweeps(self, start_angle, end_angle, distance_of_scan):

        self.sonar.set_new_range(distance_of_scan)
        sonar_request_msg = sweepGoal()
        sonar_request_msg.start_angle = start_angle
        sonar_request_msg.end_angle = end_angle
        sonar_request_msg.distance_of_scan = distance_of_scan

        while True:
            self.on_request(sonar_request_msg)
            rospy.spin()


    def convert_to_ros_compressed_msg(self, image, compressed_format='jpg'):
        """
        Convert any kind of image to ROS Compressed Image.
        """
        return self.cv_bridge.cv2_to_compressed_imgmsg(image, dst_format=compressed_format)

    def run(self):
        # If debug mode is on, do constant sweeps within range
        if self.debug:
            self.constant_sweeps(self.CONSTANT_SWEEP_START, self.CONSTANT_SWEEP_END, self.SONAR_DEFAULT_RANGE)
        else:
            rospy.Subscriber(self.SONAR_REQUEST_TOPIC, sweepGoal, self.on_request)
            rospy.loginfo("starting sonar_publisher...")
            rospy.spin()


if __name__ == '__main__':
    try:
        SonarPublisher().run()
    except rospy.ROSInterruptException:
        pass
