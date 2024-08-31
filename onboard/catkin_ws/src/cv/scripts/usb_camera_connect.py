#!/usr/bin/env python3

import rospy
import roslaunch
import yaml
import resource_retriever as rr

CAMERA_CONFIG_PATH = 'package://cv/configs/usb_cameras.yaml'


def connect_all():
    # get camera specs
    with open(rr.get_filename(CAMERA_CONFIG_PATH, use_protocol=False)) as f:
        cameras = yaml.safe_load(f)

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    roslaunch_files = []
    for camera_name in cameras:
        # add cli args for each camera to run upon init
        camera = cameras[camera_name]
        device_path = camera["device_path"]
        topic = camera["topic"]

        cli_args = ["cv", "usb_camera.launch", f"topic:={topic}", f"device_path:={device_path}", "framerate:=10"]
        roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)[0]
        roslaunch_files.append((roslaunch_file, cli_args[2:]))

    # actually init
    parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_files)
    parent.start()

    while not rospy.is_shutdown():
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node("usb_camera_connect")
    connect_all()
