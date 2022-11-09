#!/usr/bin/env python3

import rospy
import depthai as dai
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import depthai_camera_connect


class DepthAIImageStreamPublisher:
    """
    Class to stream the RGB video, RGB stereo, mono left, mono right, and disparity image feeds from the depthai camera
    and publish them to their own topics.
    """

    CAMERA = 'front'
    STREAM_TOPIC_RGB_VIDEO = f'/camera/{CAMERA}/rgb/video/stream_raw'
    STREAM_TOPIC_RGB_PREVIEW = f'/camera/{CAMERA}/rgb/preview/stream_raw'
    STREAM_TOPIC_LEFT = f'/camera/{CAMERA}/left/stream_raw'
    STREAM_TOPIC_RIGHT = f'/camera/{CAMERA}/right/stream_raw'
    STREAM_TOPIC_DISPARITY = f'/camera/{CAMERA}/disparity/stream_raw'
    STREAM_TOPIC_DEPTH = f'/camera/{CAMERA}/depth/stream_raw'

    def __init__(self):
        """
        Set up publishers and camera node pipelines.
        """
        rospy.init_node('depthai_image_stream')
        self.robot = rospy.get_param('~robot')

        self.stream_publisher_rgb_video = rospy.Publisher(self.STREAM_TOPIC_RGB_VIDEO, Image, queue_size=10)
        self.stream_publisher_rgb_preview = rospy.Publisher(self.STREAM_TOPIC_RGB_PREVIEW, Image, queue_size=10)
        self.stream_publisher_left = rospy.Publisher(self.STREAM_TOPIC_LEFT, Image, queue_size=10)
        self.stream_publisher_right = rospy.Publisher(self.STREAM_TOPIC_RIGHT, Image, queue_size=10)
        self.stream_publisher_disparity = rospy.Publisher(self.STREAM_TOPIC_DISPARITY, Image, queue_size=10)
        self.stream_publisher_depth = rospy.Publisher(self.STREAM_TOPIC_DEPTH, Image, queue_size=10)

        self.bridge = CvBridge()
        self.pipeline = dai.Pipeline()
        self.build_pipeline()

    def build_pipeline(self):
        """
        Build the DepthAI.Pipeline, which takes the camera and disparity feeds and retrieves them using XLinkOut.
        """
        camRgb = self.pipeline.create(dai.node.ColorCamera)
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        camRgb.setBoardSocket(dai.CameraBoardSocket.RGB)
        camRgb.setVideoSize(1920, 1080)
        camRgb.setPreviewSize(300, 300)
        camRgb.setInterleaved(False)
        camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)

        camLeft = self.pipeline.create(dai.node.MonoCamera)
        camLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        camLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)

        camRight = self.pipeline.create(dai.node.MonoCamera)
        camRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        camRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)

        stereo = self.pipeline.create(dai.node.StereoDepth)
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        self.stereoMaxDisparity = stereo.initialConfig.getMaxDisparity()

        xoutRgbVideo = self.pipeline.create(dai.node.XLinkOut)
        xoutRgbVideo.setStreamName("rgbVideo")
        xoutRgbVideo.input.setBlocking(False)
        xoutRgbVideo.input.setQueueSize(1)

        xoutRgbPreview = self.pipeline.create(dai.node.XLinkOut)
        xoutRgbPreview.setStreamName("rgbPreview")
        xoutRgbPreview.input.setBlocking(False)
        xoutRgbPreview.input.setQueueSize(1)

        xoutLeft = self.pipeline.create(dai.node.XLinkOut)
        xoutLeft.setStreamName("left")
        xoutLeft.input.setBlocking(False)
        xoutLeft.input.setQueueSize(1)

        xoutRight = self.pipeline.create(dai.node.XLinkOut)
        xoutRight.setStreamName("right")
        xoutRight.input.setBlocking(False)
        xoutRight.input.setQueueSize(1)

        xoutDisparity = self.pipeline.create(dai.node.XLinkOut)
        xoutDisparity.setStreamName("disparity")
        xoutDisparity.input.setBlocking(False)
        xoutDisparity.input.setQueueSize(1)

        xoutDepth = self.pipeline.create(dai.node.XLinkOut)
        xoutDepth.setStreamName("depth")
        xoutDepth.input.setBlocking(False)
        xoutDepth.input.setQueueSize(1)

        camLeft.out.link(stereo.left)
        camRight.out.link(stereo.right)

        camRgb.video.link(xoutRgbVideo.input)
        camRgb.preview.link(xoutRgbPreview.input)
        camLeft.out.link(xoutLeft.input)
        camRight.out.link(xoutRight.input)
        stereo.disparity.link(xoutDisparity.input)
        stereo.depth.link(xoutDepth.input)

    # Publish newest image off queue to topic every few seconds
    def run(self):
        """
        Get RGB video, RGB stereo, mono left, mono right, and disparity images from the camera and
        publish them to their respective topics.
        """

        with depthai_camera_connect.connect(self.pipeline, self.robot) as device:

            # Output queue, to receive message on the host from the device(you can send the message
            # on the device with XLinkOut)
            rgbVideoQueue = device.getOutputQueue(name="rgbVideo", maxSize=4, blocking=False)
            rgbPreviewQueue = device.getOutputQueue(name="rgbPreview", maxSize=4, blocking=False)
            leftQueue = device.getOutputQueue(name="left", maxSize=4, blocking=False)
            rightQueue = device.getOutputQueue(name="right", maxSize=4, blocking=False)
            disparityQueue = device.getOutputQueue(name="disparity", maxSize=4, blocking=False)
            depthQueue = device.getOutputQueue(name="depth", maxSize=4, blocking=False)

            while not rospy.is_shutdown():

                # Get messages that came from the queue
                raw_img_rgb_video = rgbVideoQueue.get()
                img_rgb_video = raw_img_rgb_video.getCvFrame()

                raw_img_rgb_preview = rgbPreviewQueue.get()
                img_rgb_preview = raw_img_rgb_preview.getCvFrame()

                raw_img_left = leftQueue.get()
                img_left = raw_img_left.getCvFrame()

                raw_img_right = rightQueue.get()
                img_right = raw_img_right.getCvFrame()

                # Normalize and apply color map to disparity image
                raw_img_disparity = disparityQueue.get()
                img_disparity = raw_img_disparity.getFrame()
                img_disparity = (img_disparity * (255 / self.stereoMaxDisparity)).astype(np.uint8)
                img_disparity = cv2.applyColorMap(img_disparity, cv2.COLORMAP_AUTUMN)

                raw_img_depth = depthQueue.get()
                img_depth = raw_img_depth.getCvFrame()

                # Publish the images
                image_msg_rgb_video = self.bridge.cv2_to_imgmsg(img_rgb_video, 'bgr8')
                self.stream_publisher_rgb_video.publish(image_msg_rgb_video)

                image_msg_rgb_preview = self.bridge.cv2_to_imgmsg(img_rgb_preview, 'bgr8')
                self.stream_publisher_rgb_preview.publish(image_msg_rgb_preview)

                image_msg_left = self.bridge.cv2_to_imgmsg(img_left, 'mono8')
                self.stream_publisher_left.publish(image_msg_left)

                image_msg_right = self.bridge.cv2_to_imgmsg(img_right, 'mono8')
                self.stream_publisher_right.publish(image_msg_right)

                image_msg_disparity = self.bridge.cv2_to_imgmsg(img_disparity, 'bgr8')
                self.stream_publisher_disparity.publish(image_msg_disparity)

                image_msg_depth = self.bridge.cv2_to_imgmsg(img_depth, 'mono16')
                self.stream_publisher_depth.publish(image_msg_depth)


if __name__ == '__main__':
    try:
        DepthAIImageStreamPublisher().run()
    except rospy.ROSInterruptException:
        pass
