#!/usr/bin/env python3

import rospy
import cv2
import time
import depthai as dai

from custom_msgs.srv import ConnectDepthAICamera, ConnectUSBCamera
import depthai_camera_connect


# Starts the `connect_depthai_camera` and `connect_usb_camera` services
# which determine whether the stereo and mono cameras are connected.

# Example usage:
# roslaunch cv camera_test_connect.launch
# rosservice call /connect_depthai_camera
# rosservice call /connect_usb_camera "channel: <channel>"
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
            return {'success': True}  # If no exception is thrown, the camera is connected

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
