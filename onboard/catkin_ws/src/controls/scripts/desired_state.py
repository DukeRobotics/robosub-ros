#!/usr/bin/env python3

import rospy
import actionlib
import controls_utils as utils
from tf import TransformListener
from pid_manager import PIDManager

from custom_msgs.msg import ControlsDesiredPoseAction, ControlsDesiredTwistAction, ControlsDesiredPowerAction


class bcolors:
    BOLD = '\033[1m'
    OKGREEN = '\033[92m'
    WARN = '\033[93m'
    FAIL = '\033[91m'
    RESET = '\033[0m'


class DesiredStateHandler:
    """Receives desired states and runs the corresponding controls algorithm.

    Attributes:
        listener: TransformListener used to transform input to local reference frame
        pid_manager: PIDManager object that is called for publishing data to PID loops
    """

    DESIRED_TWIST_ACTION = 'controls/desired_twist'
    DESIRED_POSE_ACTION = 'controls/desired_pose'
    DESIRED_POWER_ACTION = 'controls/desired_power'

    REFRESH_HZ = 10  # rate of publishing to PID topics

    # Max power & Max twist for the controls need values
    MAX_POWER = {'x': 1, 'y': 1, 'z': 1, 'roll': 0.5, 'pitch': 0.5, 'yaw': 0.5}
    MAX_TWIST = {'x': 1, 'y': 1, 'z': 1, 'roll': 0.5, 'pitch': 0.5, 'yaw': 0.5}

    event_id = 0

    def __init__(self):
        rospy.init_node('desired_state')

        self.listener = TransformListener()
        self.pid_manager = PIDManager()

        self.pose_server = actionlib.SimpleActionServer(
            self.DESIRED_POSE_ACTION,
            ControlsDesiredPoseAction,
            self._on_pose_received,
            False)
        self.twist_server = actionlib.SimpleActionServer(
            self.DESIRED_TWIST_ACTION,
            ControlsDesiredTwistAction,
            self._on_twist_received,
            False)
        self.power_server = actionlib.SimpleActionServer(
            self.DESIRED_POWER_ACTION,
            ControlsDesiredPowerAction,
            self._on_power_received,
            False)

        self.pose_server.start()
        self.twist_server.start()
        self.power_server.start()

    def _on_pose_received(self, goal):
        """Handler for receiving desired pose. Transforms global desired pose to local reference frame
        for PID loops.

        Args:
            pose: ROS Pose message corresponding to desired pose in global reference frame
        """
        self.twist_server.preempt_request = True
        self.power_server.preempt_request = True

        self._publish_control(self.pose_server, self.pid_manager.position_control,
                lambda: utils.parse_pose(utils.transform_pose(self.listener, 'odom', 'base_link', goal.pose)))

    def _on_twist_received(self, goal):
        """Handler for receiving desired twists. Received desired twists are assumed to be defined in the
        local reference frame.

        Args:
            power: ROS Twist message corresponding to desired twists in local reference frame
        """
        self.pose_server.preempt_request = True
        self.power_server.preempt_request = True

        twist = utils.parse_twist(goal.twist)

        if not self._twist_state_safety(twist):
            self.twist_server.set_aborted()
            return

        self._publish_control(self.twist_server, self.pid_manager.velocity_control, lambda: twist)

    def _on_power_received(self, goal):
        """Handler for receiving desired powers. A desired power in a given axis represents the control
        effort that the robot should exert in a certain axis (ranges from [-1, 1]). Desired power is
        local by definition.

        Args:
            power: ROS Twist message corresponding to desired powers in local reference frame
        """
        self.pose_server.preempt_request = True
        self.twist_server.preempt_request = True

        power = utils.parse_twist(goal.power)

        if not self._power_state_safety(power):
            self.power_server.set_aborted()
            return

        self._publish_control(self.power_server, self.pid_manager.power_control, lambda: power)

    def _power_state_safety(self, power):
        # Compares power with controller limits
        return_status = True
        for axis in utils.get_axes():
            if abs(power[axis]) > self.MAX_POWER[axis]:
                rospy.logerr(
                    "===> Desired power exceeds maximum %s value! Halting robot <===", axis)
                self.pid_manager.soft_estop()
                return_status = False
        return return_status

    def _twist_state_safety(self, twist):
        # Compares twist with controller limits
        return_status = True
        for axis in utils.get_axes():
            if abs(twist[axis]) > self.MAX_TWIST[axis]:
                rospy.logerr(
                    "===> Desired twist exceeds maximum %s value! Halting robot <===", axis)
                self.pid_manager.soft_estop()
                return_status = False
        return return_status

    def _publish_control(self, server, publish_func, get_data):
        rate = rospy.Rate(self.REFRESH_HZ)
        while not server.is_preempt_requested():
            publish_func(get_data())
            rate.sleep()
        self.pid_manager.soft_estop()
        server.set_preempted()


def main():
    try:
        DesiredStateHandler()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
