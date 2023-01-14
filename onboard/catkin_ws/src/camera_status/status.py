#!/usr/bin/env python3
import rospy
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class Status:
    def __init__(self):
        # Instantiate new USB camera node
        rospy.init_node('usb_camera', anonymous=True)

        # Read custom camera configs from launch command
        self.topic = rospy.get_param("~topic")
        self.topic = f'/camera/{self.topic}/image_raw'
        self.channel = rospy.get_param("~channel")
        # If no custom framerate is passed in, set self.framerate to None to trigger default framerate
        self.framerate = rospy.get_param("~framerate")

        if self.framerate == -1:
            self.framerate = None

        # Connect to usb camera
        self.cv_bridge = CvBridge()
        # Create image publisher at given topic (default /camera/image_raw)
        self.publisher = rospy.Publisher(self.topic, Image, queue_size=10)

    def publish_mono_connection_status(self):

        # Try connecting to the camera unless a connection is refused
        try:
            # Connect to camera at channel
            cap = cv2.VideoCapture(self.channel)
            cap.read()

            pub = rospy.Publisher('camera_status', std_msgs.msg.String, queue_size=10)
            pub.publish(std_msgs.msg.String("Success"))

        except Exception:
            rospy.loginfo("Camera not found at channel {self.channel}")
