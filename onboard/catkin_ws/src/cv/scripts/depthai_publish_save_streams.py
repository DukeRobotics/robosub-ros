#!/usr/bin/env python3

import rospy
import depthai as dai
import cv2
from sensor_msgs.msg import CompressedImage
from image_tools import ImageTools
import depthai_camera_connect
import numpy as np
import utils
import subprocess
import os


class DepthAIStreamsPublisherAndSaver:
    """
    Class to publish the RGB video, preview, mono left, mono right, disparity, and depth streams to respective topics.
    Also saves the RGB video, preview, mono left, mono right, and disparity streams to respective files.
    """

    # Topic names for publishing the streams
    CAMERA = 'front'
    STREAM_TOPIC_RGB_VIDEO = f'/camera/{CAMERA}/rgb/video/compressed'
    STREAM_TOPIC_RGB_PREVIEW = f'/camera/{CAMERA}/rgb/preview/compressed'
    STREAM_TOPIC_LEFT = f'/camera/{CAMERA}/left/compressed'
    STREAM_TOPIC_RIGHT = f'/camera/{CAMERA}/right/compressed'
    STREAM_TOPIC_DISPARITY = f'/camera/{CAMERA}/disparity/compressed'
    STREAM_TOPIC_DEPTH = f'/camera/{CAMERA}/depth/compressed'

    # Base path for saving the streams to files
    BASE_PATH = '/root/dev/robosub-ros/'

    def __init__(self):
        """
        Set up publisher and camera node pipeline.
        """
        rospy.init_node('depthai_publish_save_streams')

        # Framerate of the streams
        self.framerate = rospy.get_param('~framerate')

        # Whether to publish the streams to topics
        self.publish_rgb_video = rospy.get_param('~rgb_video')
        self.publish_rgb_preview = rospy.get_param('~rgb_preview')
        self.publish_left = rospy.get_param('~left')
        self.publish_right = rospy.get_param('~right')
        self.publish_disparity = rospy.get_param('~disparity')
        self.publish_depth = rospy.get_param('~depth')

        # File paths to save the streams to. If param is empty, the stream will not be saved.
        self.rgb_video_file_path = os.path.join(self.BASE_PATH, rospy.get_param('~rgb_video_file_path'))
        self.rgb_preview_file_path = os.path.join(self.BASE_PATH, rospy.get_param('~rgb_preview_file_path'))
        self.left_file_path = os.path.join(self.BASE_PATH, rospy.get_param('~left_file_path'))
        self.right_file_path = os.path.join(self.BASE_PATH, rospy.get_param('~right_file_path'))
        self.disparity_file_path = os.path.join(self.BASE_PATH, rospy.get_param('~disparity_file_path'))

        # Whether to convert the saved encoded streams into a video
        self.convert_to_video = rospy.get_param('~convert_to_video')
        # Whether to convert video to QuickTime compatible format
        self.qt_compatible = rospy.get_param('~qt_compatible')

        # Whether to save the streams to files
        self.save_rgb_video = rospy.get_param('~rgb_video_file_path') != ''
        self.save_rgb_preview = rospy.get_param('~rgb_preview_file_path') != ''
        self.save_left = rospy.get_param('~left_file_path') != ''
        self.save_right = rospy.get_param('~right_file_path') != ''
        self.save_disparity = rospy.get_param('~disparity_file_path') != ''

        num_saved_streams = (int(self.save_rgb_video) + int(self.save_rgb_preview) + int(self.save_left) +
                             int(self.save_right) + int(self.save_disparity))

        self.rgb_video_resolution = (1920, 1080)
        self.rgb_preview_resolution = (416, 416)
        self.left_resolution = (640, 400)
        self.right_resolution = (640, 400)
        self.disparity_resolution = (640, 400)

        # Sum of the number of pixels in each frame of the streams that are being saved
        encoded_pixels_per_frame = 0
        if self.save_rgb_video:
            encoded_pixels_per_frame += self.rgb_video_resolution[0] * self.rgb_video_resolution[1]
        if self.save_rgb_preview:
            encoded_pixels_per_frame += self.rgb_preview_resolution[0] * self.rgb_preview_resolution[1]
        if self.save_left:
            encoded_pixels_per_frame += self.left_resolution[0] * self.left_resolution[1]
        if self.save_right:
            encoded_pixels_per_frame += self.right_resolution[0] * self.right_resolution[1]
        if self.save_disparity:
            encoded_pixels_per_frame += self.disparity_resolution[0] * self.disparity_resolution[1]

        # Check if the framerate goes over the encoding limit
        if encoded_pixels_per_frame > 0:
            max_encoded_pixels_per_second = 3840 * 2160 * 30
            max_framerate = int(max_encoded_pixels_per_second / encoded_pixels_per_frame)
            if self.framerate > max_framerate:
                rospy.logwarn(f'Framerate {self.framerate} goes over the encoding limit. '
                              f'Using maximum possible framerate: {max_framerate}')
                self.framerate = max_framerate

        if num_saved_streams > 3:
            raise ValueError('Cannot save more than 3 streams at once')

        if self.framerate < 1 or self.framerate > 60:
            raise ValueError(f'Framerate {self.framerate} is not in range [1, 60]')

        # Check if the file paths are valid
        if self.save_rgb_video and not utils.check_file_writable(self.rgb_video_file_path):
            raise ValueError(f'RGB video file path {self.rgb_video_file_path} is not writable')

        if self.save_rgb_preview and not utils.check_file_writable(self.rgb_preview_file_path):
            raise ValueError(f'RGB preview file path {self.rgb_preview_file_path} is not writable')

        if self.save_left and not utils.check_file_writable(self.left_file_path):
            raise ValueError(f'Left file path {self.left_file_path} is not writable')

        if self.save_right and not utils.check_file_writable(self.right_file_path):
            raise ValueError(f'Right file path {self.right_file_path} is not writable')

        if self.save_disparity and not utils.check_file_writable(self.disparity_file_path):
            raise ValueError(f'Disparity file path {self.disparity_file_path} is not writable')

        # Set up publishers
        if self.publish_rgb_video:
            self.stream_publisher_rgb_video = rospy.Publisher(self.STREAM_TOPIC_RGB_VIDEO, CompressedImage,
                                                              queue_size=10)

        if self.publish_rgb_preview:
            self.stream_publisher_rgb_preview = rospy.Publisher(self.STREAM_TOPIC_RGB_PREVIEW, CompressedImage,
                                                                queue_size=10)

        if self.publish_left:
            self.stream_publisher_left = rospy.Publisher(self.STREAM_TOPIC_LEFT, CompressedImage, queue_size=10)

        if self.publish_right:
            self.stream_publisher_right = rospy.Publisher(self.STREAM_TOPIC_RIGHT, CompressedImage, queue_size=10)

        if self.publish_disparity:
            self.stream_publisher_disparity = rospy.Publisher(self.STREAM_TOPIC_DISPARITY, CompressedImage,
                                                              queue_size=10)

        if self.publish_depth:
            self.stream_publisher_depth = rospy.Publisher(self.STREAM_TOPIC_DEPTH, CompressedImage, queue_size=10)

        self.image_tools = ImageTools()
        self.pipeline = dai.Pipeline()
        self.build_pipeline()

    def build_pipeline(self):
        """
        Build the DepthAI.Pipeline, which takes the RGB camera feed and retrieves it using an XLinkOut.
        """

        # Setup ColorCamera node for RGB video/preview
        if self.publish_rgb_video or self.publish_rgb_preview or self.save_rgb_video or self.save_rgb_preview:
            camRgb = self.pipeline.create(dai.node.ColorCamera)
            camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
            camRgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)

            if self.publish_rgb_video or self.save_rgb_video:
                camRgb.setVideoSize(self.rgb_video_resolution)

            if self.publish_rgb_preview or self.save_rgb_preview:
                camRgb.setPreviewSize(self.rgb_preview_resolution)

            camRgb.setInterleaved(False)
            camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)
            camRgb.setFps(self.framerate)

        # Setup VideoEncoder and XLinkOut nodes for saving RGB video to files
        if self.save_rgb_video:
            veRgbVideo = self.pipeline.create(dai.node.VideoEncoder)
            veRgbVideo.setDefaultProfilePreset(self.framerate, dai.VideoEncoderProperties.Profile.H265_MAIN)

            xoutVeRgbVideo = self.pipeline.create(dai.node.XLinkOut)
            xoutVeRgbVideo.setStreamName("veRgbVideo")

        # Setup VideoEncoder and XLinkOut nodes for saving RGB preview to files
        if self.save_rgb_preview:
            # Must convert rgb preview to NV12 for video encoder
            manipRgbPreview = self.pipeline.create(dai.node.ImageManip)
            manipRgbPreview.initialConfig.setFrameType(dai.ImgFrame.Type.NV12)

            veRgbPreview = self.pipeline.create(dai.node.VideoEncoder)
            veRgbPreview.setDefaultProfilePreset(self.framerate, dai.VideoEncoderProperties.Profile.H265_MAIN)

            xoutVeRgbPreview = self.pipeline.create(dai.node.XLinkOut)
            xoutVeRgbPreview.setStreamName("veRgbPreview")

        # Setup MonoCamera node for left camera
        if self.publish_left or self.publish_disparity or self.publish_depth or self.save_left or self.save_disparity:
            camLeft = self.pipeline.create(dai.node.MonoCamera)
            camLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)  # Resolution: 640 x 400
            camLeft.setBoardSocket(dai.CameraBoardSocket.CAM_B)
            camLeft.setFps(self.framerate)

        # Setup VideoEncoder and XLinkOut nodes for saving left camera to files
        if self.save_left:
            veLeft = self.pipeline.create(dai.node.VideoEncoder)
            veLeft.setDefaultProfilePreset(self.framerate, dai.VideoEncoderProperties.Profile.H264_MAIN)

            xoutVeLeft = self.pipeline.create(dai.node.XLinkOut)
            xoutVeLeft.setStreamName("veLeft")

        # Setup MonoCamera node for right camera
        if self.publish_right or self.publish_disparity or self.publish_depth or self.save_right or self.save_disparity:
            camRight = self.pipeline.create(dai.node.MonoCamera)
            camRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)  # Resolution: 640 x 400
            camRight.setBoardSocket(dai.CameraBoardSocket.CAM_C)
            camRight.setFps(self.framerate)

        # Setup VideoEncoder and XLinkOut nodes for saving right camera to files
        if self.save_right:
            veRight = self.pipeline.create(dai.node.VideoEncoder)
            veRight.setDefaultProfilePreset(self.framerate, dai.VideoEncoderProperties.Profile.H264_MAIN)

            xoutVeRight = self.pipeline.create(dai.node.XLinkOut)
            xoutVeRight.setStreamName("veRight")

        # Setup StereoDepth node for disparity/depth
        if self.publish_disparity or self.publish_depth or self.save_disparity:
            stereo = self.pipeline.create(dai.node.StereoDepth)
            stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
            self.stereoMaxDisparity = stereo.initialConfig.getMaxDisparity()

        # Setup VideoEncoder and XLinkOut nodes for saving disparity to files
        if self.save_disparity:
            veDisparity = self.pipeline.create(dai.node.VideoEncoder)
            veDisparity.setDefaultProfilePreset(self.framerate, dai.VideoEncoderProperties.Profile.H265_MAIN)

            xoutVeDisparity = self.pipeline.create(dai.node.XLinkOut)
            xoutVeDisparity.setStreamName("veDisparity")

        # Setup XLinkOut nodes for publishing streams to topics
        if self.publish_rgb_video:
            xoutRgbVideo = self.pipeline.create(dai.node.XLinkOut)
            xoutRgbVideo.setStreamName("rgbVideo")
            xoutRgbVideo.input.setBlocking(False)
            xoutRgbVideo.input.setQueueSize(1)

        if self.publish_rgb_preview:
            xoutRgbPreview = self.pipeline.create(dai.node.XLinkOut)
            xoutRgbPreview.setStreamName("rgbPreview")
            xoutRgbPreview.input.setBlocking(False)
            xoutRgbPreview.input.setQueueSize(1)

        if self.publish_left:
            xoutLeft = self.pipeline.create(dai.node.XLinkOut)
            xoutLeft.setStreamName("left")
            xoutLeft.input.setBlocking(False)
            xoutLeft.input.setQueueSize(1)

        if self.publish_right:
            xoutRight = self.pipeline.create(dai.node.XLinkOut)
            xoutRight.setStreamName("right")
            xoutRight.input.setBlocking(False)
            xoutRight.input.setQueueSize(1)

        if self.publish_disparity:
            xoutDisparity = self.pipeline.create(dai.node.XLinkOut)
            xoutDisparity.setStreamName("disparity")
            xoutDisparity.input.setBlocking(False)
            xoutDisparity.input.setQueueSize(1)

        if self.publish_depth:
            xoutDepth = self.pipeline.create(dai.node.XLinkOut)
            xoutDepth.setStreamName("depth")
            xoutDepth.input.setBlocking(False)
            xoutDepth.input.setQueueSize(1)

        # Link nodes
        if self.publish_disparity or self.publish_depth or self.save_disparity:
            camLeft.out.link(stereo.left)
            camRight.out.link(stereo.right)

        if self.publish_rgb_video:
            camRgb.video.link(xoutRgbVideo.input)

        if self.publish_rgb_preview:
            camRgb.preview.link(xoutRgbPreview.input)

        if self.publish_left:
            camLeft.out.link(xoutLeft.input)

        if self.publish_right:
            camRight.out.link(xoutRight.input)

        if self.publish_disparity:
            stereo.disparity.link(xoutDisparity.input)

        if self.publish_depth:
            stereo.depth.link(xoutDepth.input)

        if self.save_rgb_video:
            camRgb.video.link(veRgbVideo.input)
            veRgbVideo.bitstream.link(xoutVeRgbVideo.input)

        if self.save_rgb_preview:
            camRgb.preview.link(manipRgbPreview.inputImage)
            manipRgbPreview.out.link(veRgbPreview.input)
            veRgbPreview.bitstream.link(xoutVeRgbPreview.input)

        if self.save_left:
            camLeft.out.link(veLeft.input)
            veLeft.bitstream.link(xoutVeLeft.input)

        if self.save_right:
            camRight.out.link(veRight.input)
            veRight.bitstream.link(xoutVeRight.input)

        if self.save_disparity:
            stereo.disparity.link(veDisparity.input)
            veDisparity.bitstream.link(xoutVeDisparity.input)

    # Publish newest image off queue to topic every few seconds
    def run(self):
        """
        Get rgb images from the camera and publish them to STREAM_TOPIC.
        """

        with depthai_camera_connect.connect(self.pipeline) as device:

            # Output queue, to receive message on the host from the device (you can send the message
            # on the device with XLinkOut)

            if self.publish_rgb_video:
                rgbVideoQueue = device.getOutputQueue(name="rgbVideo", maxSize=4, blocking=False)

            if self.publish_rgb_preview:
                rgbPreviewQueue = device.getOutputQueue(name="rgbPreview", maxSize=4, blocking=False)

            if self.publish_left:
                leftQueue = device.getOutputQueue(name="left", maxSize=4, blocking=False)

            if self.publish_right:
                rightQueue = device.getOutputQueue(name="right", maxSize=4, blocking=False)

            if self.publish_disparity:
                disparityQueue = device.getOutputQueue(name="disparity", maxSize=4, blocking=False)

            if self.publish_depth:
                depthQueue = device.getOutputQueue(name="depth", maxSize=4, blocking=False)

            if self.save_rgb_video:
                veRgbVideoQueue = device.getOutputQueue(name="veRgbVideo", maxSize=4, blocking=False)
                rgb_video_file = open(self.rgb_video_file_path + ".h265", 'wb')

            if self.save_rgb_preview:
                veRgbPreviewQueue = device.getOutputQueue(name="veRgbPreview", maxSize=4, blocking=False)
                rgb_preview_file = open(self.rgb_preview_file_path + ".h265", 'wb')

            if self.save_left:
                veLeftQueue = device.getOutputQueue(name="veLeft", maxSize=4, blocking=False)
                left_file = open(self.left_file_path + ".h264", 'wb')

            if self.save_right:
                veRightQueue = device.getOutputQueue(name="veRight", maxSize=4, blocking=False)
                right_file = open(self.right_file_path + ".h264", 'wb')

            if self.save_disparity:
                veDisparityQueue = device.getOutputQueue(name="veDisparity", maxSize=4, blocking=False)
                disparity_file = open(self.disparity_file_path + ".h265", 'wb')

            while not rospy.is_shutdown():

                # Get messages that came from the queue
                if self.publish_rgb_video:
                    raw_img_rgb_video = rgbVideoQueue.get()
                    img_rgb_video = raw_img_rgb_video.getCvFrame()
                    image_msg_rgb_video = self.image_tools.convert_to_ros_compressed_msg(img_rgb_video)
                    self.stream_publisher_rgb_video.publish(image_msg_rgb_video)

                if self.publish_rgb_preview:
                    raw_img_rgb_preview = rgbPreviewQueue.get()
                    img_rgb_preview = raw_img_rgb_preview.getCvFrame()
                    image_msg_rgb_preview = self.image_tools.convert_to_ros_compressed_msg(img_rgb_preview)
                    self.stream_publisher_rgb_preview.publish(image_msg_rgb_preview)

                if self.publish_left:
                    raw_img_left = leftQueue.get()
                    img_left = raw_img_left.getCvFrame()
                    image_msg_left = self.image_tools.convert_depth_to_ros_compressed_msg(img_left, 'mono8')
                    self.stream_publisher_left.publish(image_msg_left)

                if self.publish_right:
                    raw_img_right = rightQueue.get()
                    img_right = raw_img_right.getCvFrame()
                    image_msg_right = self.image_tools.convert_depth_to_ros_compressed_msg(img_right, 'mono8')
                    self.stream_publisher_right.publish(image_msg_right)

                if self.publish_disparity:
                    # Normalize and apply color map to disparity image
                    raw_img_disparity = disparityQueue.get()
                    img_disparity = raw_img_disparity.getFrame()
                    img_disparity = (img_disparity * (255 / self.stereoMaxDisparity)).astype(np.uint8)
                    img_disparity = cv2.applyColorMap(img_disparity, cv2.COLORMAP_AUTUMN)
                    image_msg_disparity = self.image_tools.convert_to_ros_compressed_msg(img_disparity)
                    self.stream_publisher_disparity.publish(image_msg_disparity)

                if self.publish_depth:
                    raw_img_depth = depthQueue.get()
                    img_depth = raw_img_depth.getCvFrame()
                    image_msg_depth = self.image_tools.convert_depth_to_ros_compressed_msg(img_depth, 'mono16')
                    self.stream_publisher_depth.publish(image_msg_depth)

                # Save messages to files
                while self.save_rgb_video and veRgbVideoQueue.has() and not rospy.is_shutdown():
                    veRgbVideoQueue.get().getData().tofile(rgb_video_file)

                while self.save_rgb_preview and veRgbPreviewQueue.has() and not rospy.is_shutdown():
                    veRgbPreviewQueue.get().getData().tofile(rgb_preview_file)

                while self.save_left and veLeftQueue.has() and not rospy.is_shutdown():
                    veLeftQueue.get().getData().tofile(left_file)

                while self.save_right and veRightQueue.has() and not rospy.is_shutdown():
                    veRightQueue.get().getData().tofile(right_file)

                while self.save_disparity and veDisparityQueue.has() and not rospy.is_shutdown():
                    veDisparityQueue.get().getData().tofile(disparity_file)

            # Close files
            if self.save_rgb_video:
                rgb_video_file.close()

            if self.save_rgb_preview:
                rgb_preview_file.close()

            if self.save_left:
                left_file.close()

            if self.save_right:
                right_file.close()

            if self.save_disparity:
                disparity_file.close()

        # Convert encoded video files to playable videos
        h265_convert_options = "-vcodec libx264 -pix_fmt yuv420p" if self.qt_compatible else "-c copy"

        rgb_video_command = (f"ffmpeg -framerate {self.framerate} -i {self.rgb_video_file_path}.h265 " +
                             f"{h265_convert_options} {self.rgb_video_file_path}.mp4")

        rgb_preview_command = (f"ffmpeg -framerate {self.framerate} -i {self.rgb_preview_file_path}.h265 " +
                               f"{h265_convert_options} {self.rgb_preview_file_path}.mp4")

        left_command = (f"ffmpeg -framerate {self.framerate} -i {self.left_file_path}.h264 -c copy " +
                        f"{self.left_file_path}.mp4")

        right_command = (f"ffmpeg -framerate {self.framerate} -i {self.right_file_path}.h264 -c copy " +
                         f"{self.right_file_path}.mp4")

        disparity_command = (f"ffmpeg -framerate {self.framerate} -i {self.disparity_file_path}.h265 " +
                             f"{h265_convert_options} {self.disparity_file_path}.mp4")

        if self.convert_to_video:
            rospy.loginfo("Converting encoded video files to playable videos.")
            if self.save_rgb_video:
                subprocess.Popen(rgb_video_command.split(" "))
            if self.save_rgb_preview:
                subprocess.Popen(rgb_preview_command.split(" "))
            if self.save_left:
                subprocess.Popen(left_command.split(" "))
            if self.save_right:
                subprocess.Popen(right_command.split(" "))
            if self.save_disparity:
                subprocess.Popen(disparity_command.split(" "))

        rospy.loginfo("To convert the encoded video files to a playable videos, run the following commands:")
        if self.save_rgb_video:
            rospy.loginfo(rgb_video_command)
        if self.save_rgb_preview:
            rospy.loginfo(rgb_preview_command)
        if self.save_left:
            rospy.loginfo(left_command)
        if self.save_right:
            rospy.loginfo(right_command)
        if self.save_disparity:
            rospy.loginfo(disparity_command)


if __name__ == '__main__':
    try:
        DepthAIStreamsPublisherAndSaver().run()
    except rospy.ROSInterruptException:
        pass
