#!/usr/bin/env python3

import rospy
import yaml
import resource_retriever as rr

from usb_camera import USBCamera

CAMERA_CONFIG_PATH = 'package://cv/configs/usb_cameras.yaml'


def connect_all():
    with open(rr.get_filename(CAMERA_CONFIG_PATH, use_protocol=False)) as f:
        cameras = yaml.safe_load(f)

    for _, data in cameras.items():

        channel = data["channel"]
        topic = data["topic"]

        # Set framerate to -1 to bypass reading rosparam
        USBCamera(topic=topic, channel=channel, framerate=-1)


if __name__ == "__main__":
    rospy.init_node("usb_camera_connect")
    connect_all()
