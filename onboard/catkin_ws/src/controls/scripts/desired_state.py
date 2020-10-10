#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import Float64, Bool
import controls_utils as utils


class bcolors:
    BOLD = '\033[1m'
    OKGREEN = '\033[92m'
    WARN = '\033[93m'
    FAIL = '\033[91m'
    RESET = '\033[0m'


class DesiredStateHandler:
    DESIRED_TWIST_POWER = 'controls/desired_twist_power'
    DESIRED_POSE_TOPIC = 'controls/desired_pose'

    REFRESH_HZ = 10  # for main loop

    # All variables are a dictionary with mappings between the strings in DIRECTIONS to its corresponding value
    state = {}  # Current State of the Robot
    hold = {}  # State that should be held at the start of power control
    pose = None  # Desired pose
    powers = None  # Desired power
    last_powers = None  # Power from previous loop
    # These dictionaries contain mappings between the strings in DIRECTIONS to the corresponding rospy publisher objects
    pub_pos = {}
    pub_pos_enable = {}
    pub_power = {}

    pub_vel_enable = {}

    def __init__(self):
        for d in utils.get_axes():
            self.state[d] = 0
            self.hold[d] = 0
            rospy.Subscriber(utils.get_pose_topic(d), Float64, self._on_state_received, d)
            self.pub_pos[d] = rospy.Publisher(utils.get_pid_topic(d), Float64, queue_size=3)
            self.pub_pos_enable[d] = rospy.Publisher(utils.get_pos_pid_enable(d), Bool, queue_size=3)
            self.pub_vel_enable[d] = rospy.Publisher(utils.get_vel_pid_enable(d), Bool, queue_size=3)
            self.pub_power[d] = rospy.Publisher(utils.get_power_topic(d), Float64, queue_size=3)

        rospy.Subscriber(self.DESIRED_POSE_TOPIC, Pose, self._on_pose_received)
        rospy.Subscriber(self.DESIRED_TWIST_POWER, Twist, self.on_powers_received)

    def _on_state_received(self, val, direction):
        self.state[direction] = val.data

    def _on_pose_received(self, pose):
        self.pose = utils.parse_pose(pose)

    def on_powers_received(self, twist):
        self.powers = utils.parse_twist(twist)

    def soft_estop(self):
        # Stop Moving
        utils.publish_data_constant(self.pub_pos_enable, utils.get_axes(), False)
        utils.publish_data_constant(self.pub_vel_enable, utils.get_axes(), False)
        utils.publish_data_constant(self.pub_power, utils.get_axes(), 0)
        self.powers = None
        self.last_powers = None
        self.pose = None

    def enable_loops(self):
        # Enable all PID Loops
        utils.publish_data_constant(self.pub_pos_enable, utils.get_axes(), True)
        utils.publish_data_constant(self.pub_vel_enable, utils.get_axes(), True)

    def run(self):
        rospy.init_node('desired_state')
        rate = rospy.Rate(self.REFRESH_HZ)

        warned = False
        event_id = 0

        while not rospy.is_shutdown():
            rate.sleep()

            if self.pose and self.powers:
                # More than one seen in one update cycle, so warn and continue
                rospy.logerr("===> Controls received both position and power! Halting robot. <===")
                self.soft_estop()
                continue
            elif not self.pose and not self.powers:
                self.soft_estop()
                if not warned:
                    rospy.logwarn(bcolors.WARN + ("===> Controls received neither position nor power! Halting robot. "
                                                  "(Event %d) <===" % event_id) + bcolors.RESET)
                    warned = True
                continue

            # Now we have either pose XOR powers
            if warned:
                rospy.loginfo(bcolors.OKGREEN + ("===> Controls now receiving %s (End event %d) <===" %
                                                 ("position" if self.pose else "powers", event_id)) + bcolors.RESET)
                event_id += 1
                warned = False

            if self.pose:
                self.enable_loops()
                utils.publish_data_dictionary(self.pub_pos, utils.get_axes(), self.pose)
                self.pose = None

            elif self.powers:
                if self.last_powers is None or self.powers != self.last_powers:
                    self.enable_loops()
                    # Hold Position on the current state
                    self.hold = dict(self.state)
                    self.last_powers = dict(self.powers)

                # Nonzero entries bypass PID
                # If any nonzero xy power, disable those position pid loops
                if self.powers['x'] != 0 or self.powers['y'] != 0:
                    utils.publish_data_constant(self.pub_pos_enable, ['x', 'y'], False)

                # If any nonzero roll or pitch power, disable those position pid loops
                if self.powers['roll'] != 0 or self.powers['pitch'] != 0:
                    utils.publish_data_constant(self.pub_pos_enable, ['roll', 'pitch'], False)

                # If nonzero yaw power, disable those position pid loops
                if self.powers['yaw'] != 0:
                    utils.publish_data_constant(self.pub_pos_enable, ['yaw'], False)

                # If any nonzero z power, disable those position pid loops
                if self.powers['z'] != 0:
                    utils.publish_data_constant(self.pub_pos_enable, ['z'], False)

                utils.publish_data_dictionary(self.pub_power, utils.get_axes(), self.powers)
                # Publish current state to the desired state for PID
                utils.publish_data_dictionary(self.pub_pos, utils.get_axes(), self.hold)
                self.powers = None


def main():
    try:
        DesiredStateHandler().run()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
