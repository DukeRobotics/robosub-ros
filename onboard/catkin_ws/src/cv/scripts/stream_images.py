#!/usr/bin/env python3

import rospy
import depthai as dai
import cv2
import os
from custom_msgs.srv import EnableModel
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


# Stream images from camera to host
class StreamPublisher:

    NODE_NAME = 'test_stream'
    CAMERA = 'front'
    STREAM_TOPIC = f'/camera/{CAMERA}/stream_raw'

    # Set up publisher and camera node pipeline
    def __init__(self):
        self.stream_publisher = rospy.Publisher(self.STREAM_TOPIC, Image,
                                               queue_size=10)

        self.bridge = CvBridge()
        self.pipeline = dai.Pipeline()


    # Publish newest image off queue to topic every few seconds
    def run(self):

        rospy.init_node(self.NODE_NAME)
        loop_rate = rospy.Rate(1)

        # Define source and output
        camRgb = self.pipeline.create(dai.node.ColorCamera)
        xoutRgb = self.pipeline.create(dai.node.XLinkOut)

        xoutRgb.setStreamName("rgb")

        # Properties
        camRgb.setPreviewSize(300, 300)
        camRgb.setInterleaved(False)
        camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)

        # Linking
        camRgb.preview.link(xoutRgb.input)

        # Upload the pipeline to the device
        with dai.Device(self.pipeline) as device:

            # Output queue, to receive message on the host from the device (you can send the message on the device with XLinkOut)
            rgbQueue = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
            
            while not rospy.is_shutdown():

                # Get a message that came from the queue
                raw_img = rgbQueue.tryGet()
                img = raw_img.getCvFrame()

                # Publish the image
                image_msg = self.bridge.cv2_to_imgmsg(img, 'bgr8')
                self.stream_publisher.publish(image_msg)

            loop_rate.sleep()


# Mock the camera by publishing the same image to a topic
class DummyStreamPublisher:

    NODE_NAME = 'test_stream'
    CAMERA = 'front'
    STREAM_TOPIC = f'/camera/{CAMERA}/stream_raw'

    # Read in the dummy image and other misc. setup work
    def __init__(self):
        self.stream_publisher = rospy.Publisher(self.STREAM_TOPIC, Image,
                                               queue_size=10)

        self.bridge = CvBridge()
        self.pipeline = dai.Pipeline()


    # Publish dummy image to topic every few seconds
    def run(self):

        rospy.init_node(self.NODE_NAME)
        loop_rate = rospy.Rate(1)

        # Dummy still image
        path = os.path.dirname(__file__)
        image = cv2.imread(os.path.join(path, '../assets/left384.jpg'),
                           cv2.IMREAD_COLOR)

        # Define source and output
        camRgb = self.pipeline.create(dai.node.ColorCamera)
         # Properties
        camRgb.setPreviewSize(300, 300)
        camRgb.setInterleaved(False)
        camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)

        # Point xIn to still image
        xIn = self.pipeline.create(dai.node.XLinkIn)
        xIn.setStreamName("camControl")

        manip = self.pipeline.createImageManip()
        manip.initialConfig.setResize(300, 300)
        manip.initialConfig.setFrameType(dai.ImgFrame.Type.BGR888p)
        
        xIn.out.link(manip.inputImage)

        # Upload the pipeline to the device
        with dai.Device(self.pipeline) as device:

            qCamControl = device.getInputQueue("camControl")

            # Send a message to the ColorCamera to capture a still image
            img = dai.ImgFrame()
            is_success, im_buf_arr = cv2.imencode(".jpg", image)
            byte_im = im_buf_arr.tobytes()
            img.setFrame(byte_im)
            qCamControl.send(img)

            # Output queue, to receive message on the host from the device (you can send the message on the device with XLinkOut)
            rgbQueue = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
            
            while not rospy.is_shutdown():

                # Get a message that came from the queue
                raw_img = rgbQueue.tryGet()
                img = raw_img.getCvFrame()

                # Publish the image
                image_msg = self.bridge.cv2_to_imgmsg(img, 'bgr8')
                self.stream_publisher.publish(image_msg)

            loop_rate.sleep()

if __name__ == '__main__':
    StreamPublisher().run()
