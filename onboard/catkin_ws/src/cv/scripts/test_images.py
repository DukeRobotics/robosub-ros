#!/usr/bin/env python3

import rospy
import cv2
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class DummyImagePublisher:
    """Mock the camera by publishing a still image to a topic."""

    NODE_NAME = 'test_images'
    
    def __init__(self):
        """Get command line arguments and run function appropriate for file type."""

        rospy.init_node(self.NODE_NAME)
        
        self.topic = rospy.get_param("~topic")
        self.image_publisher = rospy.Publisher(self.topic, Image, queue_size=10)

        self.feed_path = os.path.join(os.path.dirname(__file__), rospy.get_param("~feed_path"))

        self.cv_bridge = CvBridge()
    
    def run(self):
        """Runs the appropriate function based on the type of file passed in feed_path"""

        file_extension = os.path.splitext(self.feed_path)[1]
        if file_extension == '.jpg': self.run_still()
        elif file_extension == '.bag': self.run_bag()
        elif file_extension == '': self.run_folder()
        elif self.test_if_video(): self.run_video()
        else:
            raise ValueError("The feed_path passed does not have a compatible extension. The following are compatible extensions: .jpg, .bag, a folder, or a video file compatible with cv2.VideoCapture.")

    def run_still(self):
        """Publish still image to topic once per second."""

        image = cv2.imread(self.feed_path, cv2.IMREAD_COLOR)
        image_msg = self.cv_bridge.cv2_to_imgmsg(image, 'bgr8')

        loop_rate = rospy.Rate(1)

        while not rospy.is_shutdown():
            self.image_publisher.publish(image_msg)

            loop_rate.sleep()

    # TODO: Complete this function
    def run_bag(self):
        """
        Publish a simulated image feed from a rosbag file to a topic.

        Once it publishes all images in the rosbag file, it loops and publishes images from the beginning again.
        """

        # Check that the bagfile provided in self.feed_path is a valid bagfile
        # If it is not a valid bagfile, raise an exception with an appropriate error message

        # Use code inspired from remote_launch.py in the system_utils package to execute a rosbag command in the terminal
        # The rosbag command should look like this:
        # f'rosbag play {self.feed_path} -l'
        # The rosbag command should terminate when test_images.launch (this script) is terminated
    
    # TODO: Complete this function
    def run_folder(self):
        """
        Publish a simulated image feed from a folder containing images to a topic.

        All JPG images in the folder will be published in alphabetical order by filename.
        Once it publishes all JPG images in the folder, it loops and publishes images from the beginning again.
        """

        # Add an OPTIONAL framerate argument to the launch file
        
        # Get all .jpg images in the folder
        # If there are no .jpg images in the folder, raise an exception with an appropriate error message

        # while not rospy.is_shutdown():
            # Loop over all .jpg images in the folder in alphabetical order by filename with a the specified framerate
                # Parse each image using cv2.imread
                # Convert each cv2 image to ros image message using CvBridge (see run_still for an example)
                # Publish the ros image message using self.image_publisher.publish

    def test_if_video(self) -> bool:
        """Check if the file in self.feed_path is a video file that can be opened using cv2.VideoCapture."""
        
        try:
            cap = cv2.VideoCapture(self.feed_path)
            return not(cap is None or not cap.isOpened())
        except:
            return False

    def run_video(self):
        """
        Publish a simulated image feed from a AVI video file to a topic.

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
    
if __name__ == '__main__':
    try:
        DummyImagePublisher().run()
    except rospy.ROSInterruptException:
        pass
