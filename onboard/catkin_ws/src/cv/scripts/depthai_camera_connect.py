import depthai as dai
import time


def connect(pipeline, robot):
    """
    Connects to the DepthAI camera and uploads the pipeline.

    It tries to connect five times, waiting 2 seconds before trying again (so the script is successful if the camera is
    just temporarily unavailable).

    :param pipeline: The DepthAI pipeline to upload to the camera.
    :param robot: If True, connect to the camera using a method intended to be used on the robot. If False, use a method
    intended to be used locally.
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
        try:
            # If connecting on the robot, don't specify the IP address
            # If connecting locally, specify the IP address
            device = dai.Device(pipeline) if robot else dai.Device(pipeline, device_info)

            # If the execution reaches the following return statement, the line above did not raise an exception, so a
            # successful camera connection was made, and device should be returned
            return device

        except RuntimeError as e:
            # Raise the exception only on the last try
            # For all tries before the last one, don't raise the exception and try connecting again
            if i == totalTries - 1:
                raise e

        # Wait two seconds before trying again
        # This ensures the script does not terminate if the camera is just temporarily unavailable
        time.sleep(2)
