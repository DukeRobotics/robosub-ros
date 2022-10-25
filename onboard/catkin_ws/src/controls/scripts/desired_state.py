#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose, Twist
import controls_utils as utils
from tf import TransformListener
from pid_manager import PIDManager


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

    DESIRED_TWIST_TOPIC = 'controls/desired_twist'
    DESIRED_POSE_TOPIC = 'controls/desired_pose'
    DESIRED_POWER_TOPIC = 'controls/desired_power'

    REFRESH_HZ = 10  # for main loop

    # All variables are dictionaries that map directions to their corresponding value (local reference frame)
    pose = None  # Desired pose (position)
    twist = None  # Desired twist (velocity)
    power = None  # Desired power (control effort)
    event_id = 0
    valid = True

    def __init__(self):
        rospy.init_node('desired_state')

        self.listener = TransformListener()
        self.pid_manager = PIDManager()

        rospy.Subscriber(self.DESIRED_POSE_TOPIC, Pose, self._on_pose_received)
        rospy.Subscriber(self.DESIRED_TWIST_TOPIC, Twist, self._on_twist_received)
        rospy.Subscriber(self.DESIRED_POWER_TOPIC, Twist, self._on_power_received)

    def _on_pose_received(self, pose):
        """Handler for receiving desired pose. Transforms global desired pose to local reference frame
        for PID loops.

        Args:
            pose: ROS Pose message corresponding to desired pose in global reference frame
        """
        self.pose = utils.parse_pose(utils.transform_pose(self.listener, 'odom', 'base_link', pose))

    def _on_twist_received(self, twist):
        """Handler for receiving desired twists. Received desired twists are assumed to be defined in the
        local reference frame.

        Args:
            power: ROS Twist message corresponding to desired twists in local reference frame
        """
        self.twist = utils.parse_twist(twist)

    def _on_power_received(self, power):
        """Handler for receiving desired powers. A desired power in a given axis represents the control
        effort that the robot should exert in a certain axis (ranges from [-1, 1]). Desired power is
        local by definition.

        Args:
            power: ROS Twist message corresponding to desired powers in local reference frame
        """
        self.power = utils.parse_twist(power)

    def _reset_data(self):
        """Resets all desired state data"""
        self.pose = None
        self.twist = None
        self.power = None

    def _validate_status(self):
        """Validates the desired state data that was received. Status is False (invalid) if multiple desired states
        are received at the same time (i.e. position and velocity control requested) or if no desired state is received.
        If the previous status was invalid and the new status is valid, increments event_id and logs a success message.

        Returns:
            True if status is valid, false otherwise
        """
        if (self.pose and self.twist) or (self.pose and self.power) or (self.twist and self.power):
            # More than one seen in one update cycle, so warn and mark as invalid
            self.pid_manager.soft_estop()
            rospy.logerr("===> Controls received conflicting desired states! Halting robot. <===")
            return False
        elif not self.pose and not self.twist and not self.power:
            self.pid_manager.soft_estop()
            if self.valid:
                rospy.logwarn(bcolors.WARN + ("===> Controls received no desired state! Halting robot. "
                                              "(Event %d) <===" % self.event_id) + bcolors.RESET)
            return False
        elif not self.valid:
            rospy.loginfo(bcolors.OKGREEN + ("===> Controls now receiving desired state (End event %d) <===" %
                                             (self.event_id)) + bcolors.RESET)
            self.event_id += 1
        return True

    def run(self):
        rate = rospy.Rate(self.REFRESH_HZ)

        while not rospy.is_shutdown():
            rate.sleep()

            self.valid = self._validate_status()
            if self.valid:
                if self.pose:
                    self.pid_manager.position_control(self.pose)
                # TODO: Add safety net that checks that twist/power control values are within an expected range
                elif self.twist:
                    self.pid_manager.velocity_control(self.twist)
                elif self.power:
                    self.pid_manager.power_control(self.power)

            self._reset_data()


def main():
    try:
        DesiredStateHandler().run()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
