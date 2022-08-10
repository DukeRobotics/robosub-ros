#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import Float64, Bool
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
    DESIRED_TWIST_TOPIC = 'controls/desired_twist'
    DESIRED_POSE_TOPIC = 'controls/desired_pose'
    DESIRED_POWER_TOPIC = 'controls/desired_power'

    REFRESH_HZ = 10  # for main loop

    # All variables are a dictionary with mappings between the strings in DIRECTIONS to its corresponding value
    pose = None  # Desired pose
    twist = None  # Desired twist
    power = None  # Desired power

    def __init__(self):
        rospy.init_node('desired_state')

        self.listener = TransformListener()
        self.pid_manager = PIDManager()

        rospy.Subscriber(self.DESIRED_POSE_TOPIC, Pose, self._on_pose_received)
        rospy.Subscriber(self.DESIRED_TWIST_TOPIC, Twist, self._on_twist_received)
        rospy.Subscriber(self.DESIRED_POWER_TOPIC, Twist, self._on_power_received)

    def _on_pose_received(self, pose):
        self.pose = utils.parse_pose(utils.transform_pose(self.listener, 'odom', 'base_link', pose))

    def _on_twist_received(self, twist):
        self.twist = utils.parse_twist(twist)

    def _on_power_received(self, power):
        self.power = utils.parse_twist(power)

    def _reset_data(self):
        self.pose = None
        self.twist = None
        self.power = None

    def run(self):
        rate = rospy.Rate(self.REFRESH_HZ)

        warned = False
        event_id = 0

        while not rospy.is_shutdown():
            rate.sleep()

            if (self.pose and self.twist) or (self.pose and self.power) or (self.twist and self.power):
                # More than one seen in one update cycle, so warn and continue
                self.pid_manager.soft_estop()
                rospy.logerr("===> Controls received conflicting desired states! Halting robot. <===")
            elif not self.pose and not self.twist and not self.power:
                self.pid_manager.soft_estop()
                if not warned:
                    warned = True
                    rospy.logwarn(bcolors.WARN + ("===> Controls received no desired state! Halting robot. "
                                                  "(Event %d) <===" % event_id) + bcolors.RESET)
            else:
                # Now we have either pose XOR twist XOR powers
                if warned:
                    rospy.loginfo(bcolors.OKGREEN + ("===> Controls now receiving desired state (End event %d) <===" %
                                                     (event_id)) + bcolors.RESET)
                    event_id += 1
                    warned = False

                if self.pose:
                    self.pid_manager.position_control(self.pose)

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
