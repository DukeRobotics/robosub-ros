#!/usr/bin/env python

from pymba import *
import rospy
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import sys
from camera_info_manager import CameraInfoManager

class Camera:

    def __init__(self, pub, info_pub, namespace, camera_id):
        self._pub = pub
        self._info_pub = info_pub
        self._bridge = CvBridge()
        self._camera_id = camera_id
        self._namespace = namespace
        self._info_manager = self._c0 = self._frame = self._frame_data = None

    def find_camera(self, vimba):
        system = vimba.getSystem()
        system.runFeatureCommand("GeVDiscoveryAllOnce")
        rospy.sleep(0.1)

        camera_ids = vimba.getCameraIds()
        if not camera_ids:
            rospy.logerr("Sorry: Cameras were not found.")
            sys.exit(0)

        for cam_id in camera_ids:
            rospy.loginfo("Camera found: " + cam_id)

        if self._camera_id is None:
            self._camera_id = camera_ids[0]
        elif self._camera_id not in camera_ids:
            rospy.logerr("Requested camera ID (" + self._camera_id + ") not found, sorry")
            sys.exit(0)
        self._info_manager = CameraInfoManager(cname=self._camera_id, namespace=self._namespace,
                                               url="package://avt_camera/calibrations/${NAME}.yaml")
        self._info_manager.loadCameraInfo()

    def get_camera(self, vimba):
        self._c0 = vimba.getCamera(self._camera_id)
        self._c0.openCamera()

    def gigE_camera(self):
        rospy.loginfo("Packet Size: " + str(self._c0.GevSCPSPacketSize))
        rospy.loginfo("Stream Bytes Per Second: " + str(self._c0.StreamBytesPerSecond))
        self._c0.runFeatureCommand("GVSPAdjustPacketSize")
        self._c0.StreamBytesPerSecond = 100000000

    def set_pixel_format(self):
        self._c0.PixelFormat = "Mono8"
        self._c0.AcquisitionMode = "Continuous"
        self._c0.ExposureAuto = "Continuous"
        self._c0.Width = 1210
        self._c0.Height = 760
        self._frame = self._c0.getFrame()

    def initialize_camera(self, vimba):
        self.find_camera(vimba)
        self.get_camera(vimba)
        try:
            # gigE camera
            self.gigE_camera()
        except:
            # not a gigE camera
            pass
        self.set_pixel_format()

    def start_capture(self):
        self._c0.startCapture()

    def start_acquisition(self):
        self._c0.runFeatureCommand("AcquisitionStart")

    def queue_frame_capture(self):
        self._frame.queueFrameCapture()
        self._frame.waitFrameCapture()

    def get_frame_data(self):
        self._frame_data = self._frame.getBufferByteData()

    def publish_image(self, time):
        img = np.ndarray(buffer=self._frame_data,
                         dtype=np.uint8,
                         shape=(self._frame.height, self._frame.width, self._frame.pixel_bytes))
        img_message = self._bridge.cv2_to_imgmsg(img, "mono8")
        img_message.header.stamp = time
        self._pub.publish(img_message)

        ci_message = self._info_manager.getCameraInfo()
        ci_message.header.stamp = time
        self._info_pub.publish(self._info_manager.getCameraInfo())

    def stop_acquisition(self):
        self._c0.runFeatureCommand("AcquisitionStop")
        self._c0.endCapture()
        self._c0.revokeAllFrames()
        self._c0.closeCamera()
