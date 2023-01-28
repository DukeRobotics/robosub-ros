#!/usr/bin/env python3

import depthai as dai
import time
import rospy


def connect(pipeline):
    """
    Connects to the DepthAI camera and uploads the pipeline.

    It tries to connect five times, waiting 2 seconds before trying again (so the script is successful if the camera is
    just temporarily unavailable). In each try, it attempts to connect first using autodiscovery, then using manual IP
    address specification.

    :param pipeline: The DepthAI pipeline to upload to the camera.
    :return: depthai.Device object
    :raises RuntimeError: if a successful connection to the camera could not be made
    """

    # The DepthAI device that we hope to obtain
    # If device is not None, a successful connection to the camera was made
    device = None

    # IP address used for local connection
    device_info = dai.DeviceInfo("169.254.1.222")

    # Number of attempts that will be made to connect to the camera
    totalTries = 5

    for i in range(totalTries):
        if rospy.is_shutdown():
            break

        try:
            # Try connecting with autodiscovery
            device = dai.Device(pipeline)

            # If the execution reaches the following return statement, the line above did not raise an exception, so a
            # successful camera connection was made, and device should be returned
            return device

        except RuntimeError:
            pass

        if rospy.is_shutdown():
            break

        try:
            # Try connecting with manual IP address
            device = dai.Device(pipeline, device_info)

            # If the execution reaches the following return statement, the line above did not raise an exception, so a
            # successful camera connection was made, and device should be returned
            return device

        except RuntimeError as e:
            # For all tries before the last one, don't raise the exception and try connecting again
            # On the last try, raise the exception so DepthAI code doesn't run without a successful camera connection
            if i == totalTries - 1:
                raise RuntimeError((f"{totalTries} attempts were made to connect to the DepthAI camera using "
                                    "autodiscovery and manual IP address specification. All attempts failed.")) from e

        if rospy.is_shutdown():
            break

        # Wait two seconds before trying again
        # This ensures the script does not terminate if the camera is just temporarily unavailable
        wait_secs = 2
        start = time.time()
        while time.time() - start < wait_secs and not rospy.is_shutdown():
            pass

    raise RuntimeError(f"{i} attempts were made to connect to the DepthAI camera using "
                       f"autodiscovery and manual IP address specification. All attempts failed.")


if __name__ == '__main__':
    rospy.init_node('depthai_camera_connect')
    if connect(dai.Pipeline()):
        rospy.loginfo("Connected to DepthAI device successfully.")
