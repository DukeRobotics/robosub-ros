#!/usr/bin/env python3

import depthai as dai
import numpy as np
import cv2
import os
import rospy

path = os.path.dirname(__file__)
NN_PATH = os.path.join(path, '../assets/bloblol.blob')
IMAGE_RELATIVE_PATH = os.path.join(path, '../assets/left384.jpg')


class DepthAIMockDepth:
    CAMERA = 'left'
    IMAGE_TOPIC = f'/camera/{CAMERA}/image_raw'

    # Read in the dummy image and other misc. setup work
    def __init__(self):
        self.nnPath = NN_PATH
        self.pipeline = dai.Pipeline()
        self._build_pipeline()


    def _build_pipeline(self):

        

    # Publish dummy image to topic every few seconds
    def run(self, img_msg):

        # TODO: Remove this if manual IP address specification is not needed
        # Manually specify device IP address
        # https://docs.luxonis.com/projects/hardware/en/latest/pages/guides/getting-started-with-poe.html#manually-specify-device-ip
        device_info = dai.DeviceInfo("169.254.1.222")

        # Upload the pipeline to the device
        with dai.Device(self.pipeline, device_info) as device:

            


if __name__ == '__main__':
    DepthAIMockDepth()
