#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from std_srvs.srv import SetBool


class CameraRelayController:
    """ROS node that manages enabling and disabling the camera relay.

    Attributes:
        enable_service: The ROS Service used for enabling/disabling camera
        enabled: Whether camera is enabled
        ROBOT_PUB_TOPIC: The topic that thruster allocations get published to
        RUN_LOOP_RATE: The rate at which thruster allocations are published
    """

    ROBOT_PUB_TOPIC = '/offboard/camera_relay'
    RUN_LOOP_RATE = 1  # 1 Hz

    def __init__(self):
        """Initializes the ROS node.
        Relay is normally enabled, but can be disabled by a service call.
        """
        rospy.init_node('camera_relay')

        self.camera_relay_pub = rospy.Publisher(self.ROBOT_PUB_TOPIC, Bool, queue_size=3)
        self.enable_service = rospy.Service('enable_camera', SetBool, self._handle_enable_camera)

        self.enabled = True
        self.camera_relay_pub.publish(self.enabled)

    def _handle_enable_camera(self, req):
        """Handles requests to the enable ROS service, disabling/enabling output accordingly. An example call is
        `rosservice call /enable_camera true`.

        Args:
            req: The request data sent in the service call. In this case, a boolean denoting whether to enable.

        Returns:
            A message relaying the enablement status of the camera.
        """
        self.enabled = req.data
        return {'success': True, 'message': 'Successfully set enabled to ' + str(req.data)}

    def run(self):
        """Publish the camera relay status at a fixed rate."""
        rate = rospy.Rate(self.RUN_LOOP_RATE)
        while not rospy.is_shutdown():
            if not self.enabled:
                self.camera_relay_pub.publish(False)
            else:
                self.camera_relay_pub.publish(True)

            rate.sleep()


def main():
    try:
        CameraRelayController().run()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
