#!/usr/bin/env python3

import rospy
import cv2
import os
import subprocess
from sensor_msgs.msg import CompressedImage
from image_tools import ImageTools


class DummyImagePublisher:
    """Mock the camera by publishing a still image to a topic."""

    NODE_NAME = 'test_images'

    def __init__(self):
        """Get command line arguments and run function appropriate for file type."""

        rospy.init_node(self.NODE_NAME)
        self.topic = rospy.get_param("~topic")
        self.framerate = rospy.get_param("~framerate")
        self.feed_path = os.path.join(os.path.dirname(__file__), rospy.get_param("~feed_path"))

        # No feed path is passed in -- throw an expcetion
        if rospy.get_param("~feed_path") == "":
            rospy.logerr("No feed path variable given")
            rospy.spin()

        self.image_tools = ImageTools()

    def run(self):
        """Check arguments for errors and run the appropriate function based on the type of feed_path."""
        file_extension = os.path.splitext(self.feed_path)[1]

        if file_extension != ".bag":
            # Only create the publisher if feed_path is not a bagfile and self.topic exists
            if self.topic:
                self.image_publisher = rospy.Publisher(self.topic, CompressedImage, queue_size=10)
            # Raise an exception if self.topic is not specified and the feed_path is not a bagfile
            else:
                raise ValueError("A non-empty value for the topic argument must be provided \
                                    if feed_path is not a bag file.")

        # Run the appropriate function based on the file extension
        # If the file extension is not supported, raise an exception
        if self.test_if_image():
            self.run_still()
        elif file_extension == '.bag':
            self.run_bag()
        elif file_extension == '':
            self.run_folder()
        elif self.test_if_video():
            self.run_video()
        else:
            raise ValueError("The feed_path passed does not have a compatible extension. \
                                The following are compatible extensions: .jpg, .bag, a folder, \
                                or a video file compatible with cv2.VideoCapture.")

    def test_if_image(self):
        """Check if the file in self.feed_path is an image file that can be opened using cv2.imread."""

        try:
            image = cv2.imread(self.feed_path, cv2.IMREAD_COLOR)
            return not (image is None or image.size == 0)
        except Exception:
            return False

    def run_still(self):
        """Publish a still image to topic with the specified framerate."""

        image = cv2.imread(self.feed_path, cv2.IMREAD_COLOR)
        image_msg = self.image_tools.convert_to_ros_compressed_msg(image)
        loop_rate = rospy.Rate(self.framerate)
        while not rospy.is_shutdown():
            self.image_publisher.publish(image_msg)
            loop_rate.sleep()

    def run_bag(self):
        """
        Play a rosbag file with looping.

        Although this function is intended to be used with rosbag files with image feeds,
        it can be used for any rosbag file. This function does not rename the topics of the rosbag file.
        If you want to rename the topics, either manually run `rosbag play` with topic mappings
        or create a new bagfile with the topics renamed and provide it to this function.
        """
        # Keep track of the rosbag play process; will not be None if command to run rosbag is activated
        proc = None
        # Set the framerate of the stream off of what was passed in from the roslaunch command
        started = False

        while not rospy.is_shutdown():

            # Check if self.feed_path is a valid rosbag file
            if not started and os.path.isfile(self.feed_path) and self.feed_path.endswith('.bag'):
                bag_command = ['rosbag', 'play', self.feed_path, '-l']
                rospy.loginfo(bag_command)

                # Publish images from .bag file to the remapped topics
                proc = subprocess.Popen(bag_command)
                started = True

        if proc is not None:
            # Terminate the rosbag play process
            rospy.loginfo("end")
            proc.terminate()
            proc.wait()

    def run_folder(self):
        """Publish a simulated image feed from a folder containing images to a topic.

        All images in the folder readable by OpenCV will be published in alphabetical order by filename.
        Once it publishes all images in the folder, it loops and publishes images from the beginning again.
        """

        images = []
        for file in sorted(os.listdir(self.feed_path)):
            img = cv2.imread(os.path.join(self.feed_path, file), cv2.IMREAD_COLOR)
            if img is not None and img.size != 0:
                images.append(img)

        loop_rate = rospy.Rate(self.framerate)

        while not rospy.is_shutdown():
            for image in images:
                image_msg = self.image_tools.convert_to_ros_compressed_msg(image)
                self.image_publisher.publish(image_msg)

                loop_rate.sleep()

    def test_if_video(self) -> bool:
        """Check if the file in self.feed_path is a video file that can be opened using cv2.VideoCapture."""

        try:
            cap = cv2.VideoCapture(self.feed_path)
            return not (cap is None or not cap.isOpened())
        except Exception:
            return False

    def run_video(self):
        """
        Publish a simulated image feed from a video file to a topic.

        Once it publishes all images in the video file, it loops and publishes images from the beginning again.
        """
        while not rospy.is_shutdown():

            cap = cv2.VideoCapture(self.feed_path)
            success, img = cap.read()
            loop_rate = rospy.Rate(cap.get(cv2.CAP_PROP_FPS))

            # Including 'not rospy.is_shutdown()' in the loop condition here to ensure if this script is exited
            # while this loop is running, the script quits without escalating to SIGTERM or SIGKILL
            while not rospy.is_shutdown() and success:
                image_msg = self.image_tools.convert_to_ros_compressed_msg(img)
                self.image_publisher.publish(image_msg)

                success, img = cap.read()
                loop_rate.sleep()


if __name__ == '__main__':
    try:
        DummyImagePublisher().run()
    except Exception:
        exit()
