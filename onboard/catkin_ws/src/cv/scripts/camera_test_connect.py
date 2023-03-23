#!/usr/bin/env python3

import rospy
import cv2
import time
import depthai as dai

from custom_msgs.srv import ConnectDepthAICamera, ConnectUSBCamera
import depthai_camera_connect


class CameraTestConnect:

    def __init__(self):
        rospy.init_node('camera_status')

    def run(self):
        rospy.Service("connect_depthai_camera", ConnectDepthAICamera, self.connect_depthai)
        rospy.Service("connect_usb_camera", ConnectUSBCamera, self.connect_usb)
        rospy.spin()

    def connect_depthai(self, _):
        try:
            depthai_camera_connect.connect(dai.Pipeline())
            return {'success': True}

        except Exception:
            return {'success': False}

    def connect_usb(self, req):
        try:
            # Connect to camera at specified channel
            cap = cv2.VideoCapture(req.channel)

            # Attempt to successfully read a frame for 5 seconds
            time_end = time.time() + 5
            while time.time() < time_end:
                success, _ = cap.read()
                if success:
                    return {'success': True}

            return {'success': False}

        except Exception:
            return {'success': False}


if __name__ == '__main__':
    CameraTestConnect().run()
