#!/usr/bin/env python3

import rospy
import rosbag
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

    # Assume that self.feed_path is a string with the path to a rosbag file
    def run_bag(self):
        """Publish a simulated image feed from a rosbag file to a topic.

        Once it publishes all images in the rosbag file, it loops and publishes images from the beginning again.
        """
        # Keep track of the rosbag play process; will not be None if command to run rosbag is activated
        proc = None
        # Set the framerate of the stream off of what was passed in from the roslaunch command
        loop_rate = rospy.Rate(self.framerate)
        started = False

        while not rospy.is_shutdown():

            # Check if self.feed_path is a valid rosbag file
            if not started and os.path.isfile(self.feed_path) and self.feed_path.endswith('.bag'):
                
                bag_command = ['rosbag', 'play', self.feed_path, '-l']

                # Get list of topics recorded on bag
                bag_topics = rosbag.Bag(self.feed_path).get_type_and_topic_info()[1].keys()
                index = 1
                # Remap all topics to self.topic_index
                for topic in bag_topics:
                    bag_command.append(f'{topic}:={self.topic}_{index}')

                rospy.loginfo(bag_command)

                # Publish images from .bag file to the remapped topics
                proc = subprocess.Popen(bag_command)
                started = True

            loop_rate.sleep()

        if proc is not None:
            # Terminate the rosbag play process
            rospy.loginfo("end")
            proc.terminate()
            proc.wait()
    
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
    # Assume that self.feed_path is a string with the path to a folder
    def run_folder(self):
        """Publish a simulated image feed from a folder containing images to a topic.

        All JPG images in the folder will be published in alphabetical order by filename.
        Once it publishes all JPG images in the folder, it loops and publishes images from the beginning again.
        """
        # Get all .jpg images in the folder
        images = []
        for file in [os.listdir(self.feed_path).sort()]:
            # Parse each image using cv2.imread
            img = cv2.imread(file, cv2.IMREAD_COLOR)
            if img is not None:
                images.append(img)

        loop_rate = rospy.Rate(1)

        # while not rospy.is_shutdown():
        while not rospy.is_shutdown():
            # Loop over all .jpg images in the folder in alphabetical order by filename
            for image in images:
                # Convert each cv2 image to ros image message using CvBridge (see run_still for an example)
                image_msg = self.cv_bridge.cv2_to_imgmsg(image, 'bgr8')
                # Publish the ros image message using self.image_publisher.publish
                self.image_publisher.publish(image_msg)

                loop_rate.sleep()

        
if __name__ == '__main__':
    try:
        DummyImagePublisher().run()
    except rospy.ROSInterruptException:
        pass
