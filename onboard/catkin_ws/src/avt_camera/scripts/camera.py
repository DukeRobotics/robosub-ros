from pymba import *  # noqa
import rospy
from cv_bridge import CvBridge
import numpy as np
import sys
from camera_info_manager import CameraInfoManager


class Camera:

    def __init__(self, img_pub, info_pub, namespace, camera_id):
        self._img_pub = img_pub
        self._info_pub = info_pub
        self._bridge = CvBridge()
        self._camera_id = camera_id
        self._namespace = namespace
        self._info_manager = None
        self._c0 = None

    def find_camera(self):
        camera_ids = Vimba.camera_ids()
        if not camera_ids:
            rospy.logerr("Cameras were not found.")
            sys.exit(1)

        for cam_id in camera_ids:
            rospy.loginfo("Camera found: " + cam_id)

        if self._camera_id is None:
            self._camera_id = camera_ids[0]
        elif self._camera_id not in camera_ids:
            rospy.logerr(f"Requested camera ID {self._camera_id} not found")
            sys.exit(1)

        self._info_manager = CameraInfoManager(cname=self._camera_id, namespace=self._namespace,
                                               url=f"package://avt_camera/calibrations/{self._camera_id}.yaml")

    def get_camera(self, vimba):
        if self._camera_id in vimba._cameras:
            rospy.logerr("Requested camera is already in use")
            sys.exit(1)
        self._c0 = vimba.camera(self._camera_id)
        self._c0.open(adjust_packet_size=True)

    def set_pixel_format(self):
        self._c0.StreamBytesPerSecond = 100000000
        self._c0.PixelFormat = "RGB8Packed"
        self._c0.AcquisitionMode = "Continuous"
        self._c0.ExposureAuto = "Continuous"
        self._c0.Width = 1210
        self._c0.Height = 760

    def initialize_camera(self, vimba):
        self.find_camera()
        self.get_camera(vimba)
        self.set_pixel_format()

    def start_capture(self):
        self._c0.arm('Continuous', self.publish_image)
        self._c0.start_frame_acquisition()

    def publish_image(self, frame):
        rospy.loginfo("Received frame. Publishing to ROS")
        img_message = self._bridge.cv2_to_imgmsg(frame.buffer_data_numpy(), "rgb8")
        img_message.header.stamp = rospy.Time.now()
        self._img_pub.publish(img_message)
        self._info_pub.publish(self._info_manager.getCameraInfo())

    def stop_capture(self):
        self._c0.stop_frame_acquisition()
        self._c0.disarm()
        self._c0.close()
