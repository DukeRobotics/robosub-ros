#!/usr/bin/env python3

import rospy
import cv2

from sensor_msgs.msg import CompressedImage
from image_tools import ImageTools


class USBCamera:
    """
    Object to stream any camera at /dev/video* and publishes the image feed at the device framerate
                                    (currently used for the deepwater exploration usb mono cameras)

    Launch using: roslaunch cv usb_camera.launch
    :param topic: rostopic to publish the image feed to; default is set to camera/usb_camera/compressed
    :param device_path: path to device to read the stream from (e.g., /dev/video0); can be a symlinked path
    :param framerate: custom framerate to stream the camera at; default is set to device default
    """

    def __init__(self, topic=None, device_path=None, framerate=None):
        # Instantiate new USB camera node
        rospy.init_node(f'usb_camera_{topic}', anonymous=True)

        # Read custom camera configs from launch command
        self.topic = topic if topic else rospy.get_param("~topic")
        self.topic = f'/camera/usb/{self.topic}/compressed'

        self.device_path = device_path if device_path else rospy.get_param("~device_path")

        # If no custom framerate is passed in, set self.framerate to None to trigger default framerate
        self.framerate = framerate if framerate else rospy.get_param("~framerate")

        if self.framerate == -1:
            self.framerate = None

        # Create image publisher at given topic
        self.publisher = rospy.Publisher(self.topic, CompressedImage, queue_size=10)

        self.image_tools = ImageTools()

    def run(self):
        """
        Connect to camera found at self.device_path using cv2.VideoCaptures
        and stream every image as it comes in at the device framerate
        """

        total_tries = 5
        success = False

        for _ in range(total_tries):
            if rospy.is_shutdown():
                break

            # Try connecting to the camera unless a connection is refused
            try:
                # Connect to camera at device_path
                cap = cv2.VideoCapture(self.device_path)
                # Read first frame
                success, img = cap.read()

                # Set publisher rate (framerate) to custom framerate if specified, otherwise, set to default
                loop_rate = None
                if self.framerate is None:
                    loop_rate = rospy.Rate(cap.get(cv2.CAP_PROP_FPS))
                else:
                    loop_rate = rospy.Rate(self.framerate)

                # If the execution reaches the following statement, and the first frame was successfully read,
                # then a successful camera connection was made, and we enter the main while loop
                if success:
                    break

            except Exception:
                rospy.loginfo("Failed to connect to USB camera, trying again...")
                pass

            if rospy.is_shutdown():
                break

            # Wait two seconds before trying again
            # This ensures the script does not terminate if the camera is just temporarily unavailable
            rospy.sleep(2)

        if success:
            # Including 'not rospy.is_shutdown()' in the loop condition here to ensure if this script is exited
            # while this loop is running, the script quits without escalating to SIGTERM or SIGKILL
            while not rospy.is_shutdown():
                if success:
                    # Convert image read from cv2.videoCapture to image message to be published
                    image_msg = self.image_tools.convert_to_ros_compressed_msg(img)  # Compress image
                    # Publish the image
                    self.publisher.publish(image_msg)

                # Read next image
                success, img = cap.read()
                # Sleep loop to maintain frame rate
                loop_rate.sleep()
        else:
            rospy.logerr(f"{total_tries} attempts were made to connect to the USB camera. "
                         f"The camera was not found at device_path {self.device_path}. All attempts failed.")
            raise RuntimeError(f"{total_tries} attempts were made to connect to the USB camera. "
                               f"The camera was not found at device_path {self.device_path}. All attempts failed.")


if __name__ == '__main__':
    try:
        USBCamera().run()
    except Exception:
        rospy.logerr("USB camera run failed!")
        exit()
