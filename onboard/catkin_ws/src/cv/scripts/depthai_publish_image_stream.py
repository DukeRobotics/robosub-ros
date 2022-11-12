#!/usr/bin/env python3

import rospy
import depthai as dai
import cv2
import os
from custom_msgs.srv import EnableModel
from sensor_msgs.msg import Image
from cv_bridge import CvBridge



class DepthAIImageStreamPublisher:
    """
    Class to stream the RGB image feed from the depthai camera and publish the raw feed to STREAM_TOPIC.
    """

    CAMERA = 'front'
    STREAM_TOPIC = f'/camera/{CAMERA}/stream_raw'

    def __init__(self):
        """
        Set up publisher and camera node pipeline.
        """
        rospy.init_node('depthai_image_stream')
        self.stream_publisher = rospy.Publisher(self.STREAM_TOPIC, Image,
                                               queue_size=10)

        self.bridge = CvBridge()
        self.pipeline = dai.Pipeline()
        self.build_pipeline()

    def build_pipeline(self):
        """
        Build the DepthAI.Pipeline, which takes the RGB camera feed and retrieves it using an XLinkOut.
        """
        camRgb = self.pipeline.create(dai.node.ColorCamera)
        camRgb.setPreviewSize(300, 300)
        camRgb.setInterleaved(False)
        camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)

        xoutRgb = self.pipeline.create(dai.node.XLinkOut)
        xoutRgb.setStreamName("rgb")

        camRgb.preview.link(xoutRgb.input)

    # Publish newest image off queue to topic every few seconds
    def run(self):
        """
        Get rgb images from the camera and publish them to STREAM_TOPIC.
        """
        loop_rate = rospy.Rate(1)

        # TODO: Remove this if manual IP address specification is not needed
        # Manually specify device IP address
        # https://docs.luxonis.com/projects/hardware/en/latest/pages/guides/getting-started-with-poe.html#manually-specify-device-ip
        device_info = dai.DeviceInfo("169.254.1.222")

        # NOTE:
        # If you get an error along the lines of:
        # "Could not load the Qt platform plugin "xcb" in "/usr/local/lib/python3.8/dist-packages/cv2/qt/plugins" even though it was found. 
        # This application failed to start because no Qt platform plugin could be initialized. Reinstalling the application may fix this problem."
        # This error occurs because the container thinks there no UI to show the graphics window. 
        # This is because you SSH'd into the onboard container WITHOUT specifying the -X or -Y flag, which enables graphics forwarding.
        # To fix this error, exit your current SSH session, and SSH into the onboard container again with the -X or -Y flag and it should work.

        # Upload the pipeline to the device
        with dai.Device(self.pipeline, device_info) as device:

            # Output queue, to receive message on the host from the device (you can send the message on the device with XLinkOut)
            rgbQueue = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
            
            while not rospy.is_shutdown():

                # Get a message that came from the queue
                raw_img = rgbQueue.get()
                img = raw_img.getCvFrame()

                # Publish the image
                image_msg = self.bridge.cv2_to_imgmsg(img, 'bgr8')
                self.stream_publisher.publish(image_msg)

            loop_rate.sleep()

if __name__ == '__main__':
    try:
        DepthAIImageStreamPublisher().run()
    except rospy.ROSInterruptException:
        pass