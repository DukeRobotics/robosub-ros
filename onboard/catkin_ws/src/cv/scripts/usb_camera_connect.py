#!/usr/bin/env python3

import rospy
import yaml
import resource_retriever as rr

from usb_camera import USBCamera

CAMERA_CONFIG_PATH = 'package://cv/configs/usb_cameras.yaml'


def connect_all():
    with open(rr.get_filename(CAMERA_CONFIG_PATH, use_protocol=False)) as f:
        cameras = yaml.safe_load(f)

    for camera_name in cameras:
        camera = cameras[camera_name]
        channel = camera["channel"]
        topic = camera["topic"]

        # Set framerate to -1 to bypass reading rosparam
        USBCamera(topic, channel, -1).run()


if __name__ == "__main__":
    rospy.init_node("usb_camera_connect")
    connect_all()
