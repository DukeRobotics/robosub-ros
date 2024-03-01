#!/usr/bin/env python3

import depthai as dai
import rospy
import nmap
import resource_retriever as rr
import yaml

CAMERA_CONFIG_PATH = 'package://cv/configs/cameras.yaml'


def connect(pipeline, camera_name="front"):
    """
    Connects to the DepthAI camera and uploads the pipeline.

    It tries to connect five times, waiting 2 seconds before trying again (so the script is successful if the camera is
    just temporarily unavailable). In each try, it attempts to connect first using custom autodiscovery, then using
    DepthAI autodiscovery, then finally using static IP address specification.

    :param pipeline: The DepthAI pipeline to upload to the camera.
    :return: depthai.Device object
    :raises RuntimeError: if a successful connection to the camera could not be made
    """
    with open(rr.get_filename(CAMERA_CONFIG_PATH, use_protocol=False)) as f:
        cameras = yaml.safe_load(f)

    if (config := cameras.get(camera_name)) is None:
        raise RuntimeError(f"Camera {camera_name} not found in {CAMERA_CONFIG_PATH}.")

    # The DepthAI device that we hope to obtain
    # If device is not None, a successful connection to the camera was made
    device = None

    # # IP address used for local connection
    # device_info = dai.DeviceInfo("169.254.1.222")

    # Number of attempts that will be made to connect to the camera
    total_tries = 5

    for _ in range(total_tries):
        if rospy.is_shutdown():
            break

        if (mac_address := config.get("mac_address")):
            try:
                # Scan for camera IP address using custom autodiscovery
                ip = custom_autodiscovery(mac_address)
                # Try connecting with the discovered IP address
                device = dai.Device(pipeline, dai.DeviceInfo(ip))

                # If the execution reaches the following return statement, the lines above did not raise an exception,
                # so a successful camera connection was made, and device should be returned
                return device

            except RuntimeError:
                pass

            if rospy.is_shutdown():
                break

            try:
                # Try connecting with DepthAI autodiscovery
                device = dai.Device(pipeline)

                # If the execution reaches the following return statement, the line above did not raise an exception,
                # so a successful camera connection was made, and device should be returned
                return device

            except RuntimeError:
                pass

        if rospy.is_shutdown():
            break

        # Wait two seconds before trying again
        # This ensures the script does not terminate if the camera is just temporarily unavailable
        rospy.sleep(2)

    raise RuntimeError(f"{total_tries} attempts were made to connect to the DepthAI camera using "
                       f"autodiscovery and static IP address specification. All attempts failed.")


def custom_autodiscovery(mac_address: str):
    """
    Scans all IP addresses from 192.168.1.0 to 192.168.1.255 looking for the DepthAI camera's MAC address.

    :return: DepthAI IP address string
    :raises RuntimeError: if the camera's IP address could not be found
    """

    IP_range = "192.168.1.0/24"  # 192.168.1.0 to 192.168.1.255

    nm = nmap.PortScanner()
    scan = nm.scan(hosts=IP_range, arguments='-sP')['scan']

    for ip, info in scan.items():
        if info['status']['state'] == 'up' and info['addresses'].get('mac') == mac_address:
            return ip

    raise RuntimeError("Custom autodiscovery failed to find camera.")


if __name__ == "__main__":
    rospy.init_node("depthai_camera_connect")
    camera_name = rospy.get_param("~camera")
    if connect(dai.Pipeline(), camera_name):
        rospy.loginfo("Connected to DepthAI device successfully.")
