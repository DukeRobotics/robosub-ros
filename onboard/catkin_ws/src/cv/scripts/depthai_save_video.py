#!/usr/bin/env python3

import depthai as dai
import depthai_camera_connect
import rospy
import os


class DepthAIVideoSaver:
    """
    Class to save the RGB and mono image feeds from the depthai camera.
    """

    CAMERA = 'front'

    def __init__(self):
        rospy.init_node('depthai_save_video')

        self.rgb_video_file_name = os.path.join(os.path.dirname(__file__), rospy.get_param('~rgb_video_file_name'))
        self.left_file_name = os.path.join(os.path.dirname(__file__), rospy.get_param('~left_file_name'))
        self.right_file_name = os.path.join(os.path.dirname(__file__), rospy.get_param('~right_file_name'))

        self.save_rgb_video = rospy.get_param('~rgb_video_file_name') != ""
        self.save_left = rospy.get_param('~left_file_name') != ""
        self.save_right = rospy.get_param('~right_file_name') != ""

        if self.save_rgb_video and not self.check_file_writable(self.rgb_video_file_name):
            raise ValueError(f'Cannot write to {self.rgb_video_file_name}')

        if self.save_left and not self.check_file_writable(self.left_file_name):
            raise ValueError(f'Cannot write to {self.left_file_name}')

        if self.save_right and not self.check_file_writable(self.right_file_name):
            raise ValueError(f'Cannot write to {self.right_file_name}')

        self.pipeline = dai.Pipeline()
        self.build_pipeline()

    def check_file_writable(self, filename):
        """
        Check if a file can be created or overwritten.
        """
        if os.path.exists(filename):
            # path exists
            if os.path.isfile(filename):
                # also works when file is a link and the target is writable
                return os.access(filename, os.W_OK)
            else:
                # path is a dir, so cannot write as a file
                return False
        # target does not exist, check perms on parent dir
        pdir = os.path.dirname(filename)
        if not pdir:
            pdir = '.'
        # target is creatable if parent dir is writable
        return os.access(pdir, os.W_OK)

    def build_pipeline(self):
        """
        Build the DepthAI.Pipeline, which takes the raw image feeds from the camera, encodes it into H.265,
        and retrieves it using an XLinkOut
        """

        framerate = 30

        if self.save_rgb_video:
            camRgb = self.pipeline.create(dai.node.ColorCamera)
            camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
            camRgb.setBoardSocket(dai.CameraBoardSocket.RGB)
            camRgb.setVideoSize(1920, 1080)
            camRgb.setInterleaved(False)
            camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)

            veRgb = self.pipeline.create(dai.node.VideoEncoder)
            veRgb.setDefaultProfilePreset(framerate, dai.VideoEncoderProperties.Profile.H265_MAIN)

            veRgbOut = self.pipeline.create(dai.node.XLinkOut)
            veRgbOut.setStreamName('veRgbOut')

            camRgb.video.link(veRgb.input)
            veRgb.bitstream.link(veRgbOut.input)

        if self.save_left:
            camLeft = self.pipeline.create(dai.node.MonoCamera)
            camLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
            camLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)

            veLeft = self.pipeline.create(dai.node.VideoEncoder)
            veLeft.setDefaultProfilePreset(framerate, dai.VideoEncoderProperties.Profile.H264_MAIN)

            veLeftOut = self.pipeline.create(dai.node.XLinkOut)
            veLeftOut.setStreamName('veLeftOut')

            camLeft.out.link(veLeft.input)
            veLeft.bitstream.link(veLeftOut.input)

        if self.save_right:
            camRight = self.pipeline.create(dai.node.MonoCamera)
            camRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
            camRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)

            veRight = self.pipeline.create(dai.node.VideoEncoder)
            veRight.setDefaultProfilePreset(framerate, dai.VideoEncoderProperties.Profile.H264_MAIN)

            veRightOut = self.pipeline.create(dai.node.XLinkOut)
            veRightOut.setStreamName('veRightOut')

            camRight.out.link(veRight.input)
            veRight.bitstream.link(veRightOut.input)

    def run(self):
        """
        Run the DepthAI pipeline, and save the video feeds to the specified files.
        """
        with depthai_camera_connect.connect(self.pipeline) as device:
            if self.save_rgb_video:
                veRgbOut = device.getOutputQueue(name='veRgbOut', maxSize=30, blocking=True)
                rgb_video_file = open(self.rgb_video_file_name + ".h265", 'wb')

            if self.save_left:
                veLeftOut = device.getOutputQueue(name='veLeftOut', maxSize=30, blocking=True)
                left_file = open(self.left_file_name + ".h264", 'wb')

            if self.save_right:
                veRightOut = device.getOutputQueue(name='veRightOut', maxSize=30, blocking=True)
                right_file = open(self.right_file_name + ".h264", 'wb')

            while not rospy.is_shutdown():
                while self.save_rgb_video and veRgbOut.has() and not rospy.is_shutdown():
                    veRgbOut.get().getData().tofile(rgb_video_file)

                while self.save_left and veLeftOut.has() and not rospy.is_shutdown():
                    veLeftOut.get().getData().tofile(left_file)

                while self.save_right and veRightOut.has() and not rospy.is_shutdown():
                    veRightOut.get().getData().tofile(right_file)

            if self.save_rgb_video:
                rgb_video_file.close()

            if self.save_left:
                left_file.close()

            if self.save_right:
                right_file.close()

            # To convert the H.264/5 files to a playable videos, run the following command for each file:
            # ffmpeg -framerate 25 -i <video_file_name>.h264/5 -c copy <video_file_name>.mp4
            #
            # QuickTime player will not be able to view the RGB video, but VLC will.
            # The left and right videos can be viewed in QuickTime player.
            #
            # To make the RGB video viewable in QuickTime player, run the following command:
            # fmpeg -framerate 25 -i rgb.h265 -vcodec libx264 -pix_fmt yuv420p rgb.mp4
            #
            # This command is quite slow as it converts the video encoding to be compatible with QuickTime player.
            # It is generally preferrable to use VLC to view the RGB video generated by the first command.


if __name__ == '__main__':
    try:
        DepthAIVideoSaver().run()
    except rospy.ROSInterruptException:
        pass
