#!/usr/bin/env python3

import rospy
import cv2
import os
import subprocess
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class DummyImagePublisher:
    """Mock the camera by publishing a still image to a topic."""

    NODE_NAME = 'test_images'
    
    def __init__(self):
        """Get command line arguments and run function appropriate for file type."""

        rospy.init_node(self.NODE_NAME)
        
        self.topic = rospy.get_param("~topic")
        self.framerate = rospy.get_param("~framerate")
        self.image_publisher = rospy.Publisher(self.topic, Image, queue_size=10)

        self.feed_path = os.path.join(os.path.dirname(__file__), rospy.get_param("~feed_path"))

        self.cv_bridge = CvBridge()
    
    def run(self):
        """Runs the appropriate function based on the type of file passed in feed_path"""

        file_extension = os.path.splitext(self.feed_path)[1]
        if file_extension == '.jpg': self.run_still()
        elif file_extension == '.bag': self.run_bag()
        elif file_extension == '.avi': self.run_avi()
        elif file_extension == '.mov': self.run_mov()
        elif file_extension == '': self.run_folder()
        else:
            raise ValueError("The feed_path passed does not have a compatible extension. The following are compatible extensions: .jpg, .bag, .avi, .mov, or a folder.")

    def run_still(self):
        """Publish still image to topic once per second."""

        image = cv2.imread(self.feed_path, cv2.IMREAD_COLOR)
        image_msg = self.cv_bridge.cv2_to_imgmsg(image, 'bgr8')

        loop_rate = rospy.Rate(1)

        while not rospy.is_shutdown():
            self.image_publisher.publish(image_msg)

            loop_rate.sleep()

    # TODO: Complete this function
    # Assume that self.feed_path is a string with the path to a rosbag file
    def run_bag(self):
        """Publish a simulated image feed from a rosbag file to a topic.

        Once it publishes all images in the rosbag file, it loops and publishes images from the beginning again.
        """
        while not rospy.is_shutdown():
            if self.feed_path:
                # Check if self.feed_path is a valid rosbag file
                pass
            proc = subprocess.Popen(f'rosbag play {self.feed_path} -l')
            pass
    
    def run_avi(self):
        """Publish a simulated image feed from a AVI video file to a topic.

        Once it publishes all images in the AVI file, it loops and publishes images from the beginning again.
        """
        while not rospy.is_shutdown():

            cap = cv2.VideoCapture(self.feed_path)
            success, img = cap.read()
            loop_rate = rospy.Rate(cap.get(cv2.CAP_PROP_FPS))

            while success:
                rospy.loginfo(cap.get(cv2.CAP_PROP_FPS))
                image_msg = self.cv_bridge.cv2_to_imgmsg(img, 'bgr8')
                self.image_publisher.publish(image_msg)

                success, img = cap.read()
                loop_rate.sleep()

    # TODO: Complete this function
    # Assume that self.feed_path is a string with the path to a MOV video file
    def run_mov(self):
        """Publish a simulated image feed from a MOV video file to a topic.

        Once it publishes all images in the MOV file, it loops and publishes images from the beginning again.
        """
    
    # TODO: Complete this function
    # Assume that self.feed_path is a string with the path to a folder
    def run_folder(self):
        """Publish a simulated image feed from a folder containing images to a topic.

        All JPG images in the folder will be published in alphabetical order by filename.
        Once it publishes all JPG images in the folder, it loops and publishes images from the beginning again.
        """
        # Get all .jpg images in the folder

        # while not rospy.is_shutdown():
            # Loop over all .jpg images in the folder in alphabetical order by filename
                # Parse each image using cv2.imread
                # Convert each cv2 image to ros image message using CvBridge (see run_still for an example)
                # Publish the ros image message using self.image_publisher.publish

        
if __name__ == '__main__':
    try:
        DummyImagePublisher().run()
    except rospy.ROSInterruptException:
        pass
