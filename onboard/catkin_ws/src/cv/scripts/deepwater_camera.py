#!/usr/bin/env python3

import rospy
import cv2
import os

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class DeepwaterCamera:

    def __init__(self):

        # Instantiate new deepwater camera node
        rospy.init_node('cv', anonymous=True)

        # Read custom camera configs
        self.topic = rospy.get_param("~topic")
        self.channel = rospy.get_param("~channel")
        self.framerate = rospy.get_param("~framerate")

        # Connect to current camera
        self.cv_bridge = CvBridge()
        # Create image publisher
        self.publisher = rospy.Publisher(self.topic, Image, queue_size=10)

    def run(self):
        while not rospy.is_shutdown():
            try:
                cap = cv2.VideoCapture(self.channel)
                loop_rate = rospy.Rate(cap.get(cv2.CAP_PROP_FPS))

                # Including 'not rospy.is_shutdown()' in the loop condition here to ensure if this script is exited
                # while this loop is running, the script quits without escalating to SIGTERM or SIGKILL
                while not rospy.is_shutdown() and success:
                    image_msg = self.cv_bridge.cv2_to_imgmsg(img, 'bgr8')
                    self.publisher.publish(image_msg)

                    success, img = cap.read()
                    loop_rate.sleep()
            except:
                rospy.loginfo("Camera not found at channel {self.channel}")

if __name__ == '__main__':
    try:
        DeepwaterCamera().run()
    except Exception:
        exit()