#!/usr/bin/env python3

import rospy
import cv2

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from utils import ImageTools


class USBCamera:
    """
    Object to stream any camera at /dev/video* and publishes the image feed at the device framerate
                                    (currently used for the deepwater exploration usb mono cameras)

    Launch using: roslaunch cv usb_camera.launch
    :param topic: rostopic to publish the image feed to; default is set to camera/image_raw/compressed
    :param channel: which device channel to read the stream from (e.g., dev/video0)
    :param framerate: custom framerate to stream the camera at; default is set to device default
    """

    def __init__(self):

        # Instantiate new USB camera node
        rospy.init_node('usb_camera', anonymous=True)

        # Read custom camera configs from launch command
        self.topic = rospy.get_param("~topic")
        self.topic = f'/camera/{self.topic}/image_raw/compressed'
        self.channel = rospy.get_param("~channel")
        # If no custom framerate is passed in, set self.framerate to None to trigger default framerate
        self.framerate = rospy.get_param("~framerate")

        if self.framerate == -1:
            self.framerate = None

        # Connect to usb camera
        self.cv_bridge = CvBridge()
        # Create image publisher at given topic (default /camera/image_raw/compressed)
        self.publisher = rospy.Publisher(self.topic, CompressedImage, queue_size=10)

    def run(self):
        """
        Connect to camera found at self.channel using cv2.VideoCaptures
        and stream every image as it comes in at the device framerate
        """

        # Try connecting to the camera unless a connection is refused
        try:
            # Connect to camera at channel
            cap = cv2.VideoCapture(self.channel)
            # Read first frame
            success, img = cap.read()

            # Set publisher rate (framerate) to custom framerate if specified, otherwise, set to default
            loop_rate = None
            if self.framerate is None:
                loop_rate = rospy.Rate(cap.get(cv2.CAP_PROP_FPS))
            else:
                loop_rate = rospy.Rate(self.framerate)

            # Including 'not rospy.is_shutdown()' in the loop condition here to ensure if this script is exited
            # while this loop is running, the script quits without escalating to SIGTERM or SIGKILL
            while not rospy.is_shutdown() and success:
                # Convert image read from cv2.videoCapture to image message to be published
                image_msg = ImageTools().convert_to_ros_compressed_msg(img)  # Compress image
                # Publish the image
                self.publisher.publish(image_msg)

                # Read next image
                success, img = cap.read()
                # Sleep loop to maintain frame rate
                loop_rate.sleep()
        except Exception:
            rospy.loginfo("Camera not found at channel {self.channel}")


if __name__ == '__main__':
    try:
        USBCamera().run()
    except Exception:
        rospy.logerr("USB camera run failed!")
        exit()
