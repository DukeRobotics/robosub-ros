#!/usr/bin/env python

import rospy
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Pose, Twist, Vector3
import drc_utils as utils

class bcolors:
    BOLD = '\033[1m'
    OKGREEN = '\033[92m'
    WARN = '\033[93m'
    FAIL = '\033[91m'
    RESET = '\033[0m'

class DesiredStateHandler():

    DIRECTIONS = ['x', 'y', 'z', 'roll', 'pitch', 'yaw']

    DESIRED_TWIST_POWER = 'controls/desired_twist_power'
    DESIRED_POSE_TOPIC = 'controls/desired_pose'

    REFRESH_HZ = 10  # for main loop

    # All variables are a dictionary with mappings between the strings in DIRECTIONS to its corresponding value
    vals = {}
    hold = {}
    pose = None
    powers = None
    last_powers = None
    # These dictionaries contain mappings between the strings in DIRECTION to the corresponding rospy publisher objects
    pub_pos = {}
    pub_pos_enable = {}
    pub_power = {}

    def __init__(self):
        for d in self.DIRECTIONS:
            self.vals[d] = 0
            self.hold[d] = 0
            rospy.Subscriber(utils.get_pose_topic(d), Float64, self.receive, d)
            self.pub_pos[d] = rospy.Publisher(utils.get_pid_topic(d), Float64, queue_size=3)
            self.pub_pos_enable[d] = rospy.Publisher(utils.get_pid_enable(d), Bool, queue_size=3)
            self.pub_power[d] = rospy.Publisher(utils.get_power_topic(d), Float64, queue_size=3)

        rospy.Subscriber(self.DESIRED_POSE_TOPIC, Pose, self.receive_pose)
        rospy.Subscriber(self.DESIRED_TWIST_POWER, Twist, self.receive_powers)

    def receive(self, val, direction):
        self.vals[direction] = val.data

    def receive_pose(self, pose):
        self.pose = utils.parse_pose(pose)

    def receive_powers(self, twist):
        self.powers = utils.parse_twist(twist)

    def soft_estop(self):
        #Stop Moving
        utils.publish_data_constant(self.pub_pos_enable, self.DIRECTIONS, False)
        utils.publish_data_constant(self.pub_power, self.DIRECTIONS, False)
        self.powers = None
        self.last_powers = None
        self.pose = None

    def enable_loops(self):
        #Enable all PID Loops
        utils.publish_data_constant(self.pub_pos_enable, self.DIRECTIONS, True)

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
                    rospy.logwarn(bcolors.WARN + ("===> Controls received neither position nor power! Halting robot. (Event %d) <===" % event_id) + bcolors.RESET)
                    warned = True
                continue

            # Now we have either pose XOR powers
            if warned:
                rospy.loginfo(bcolors.OKGREEN + ("===> Controls now receiving %s (End event %d) <===" % ("position" if self.pose else "powers", event_id)) + bcolors.RESET)
                event_id += 1
                warned = False

            if self.pose:
                self.enable_loops()
                self.vals = dict(self.pose)
                utils.publish_data_dictionary(self.pub_pos, self.DIRECTIONS, self.vals)
                self.pose = None

            elif self.powers:
                if self.last_powers is None or self.powers != self.last_powers:
                    self.enable_loops()
                    # Hold Position on the current state
                    self.hold = dict(self.vals)
                    self.last_powers = dict(self.powers)

                # Nonzero entries bypass PID
                # If any nonzero xy power, disable those position pid loops
                if self.powers['x'] != 0 or self.powers['y'] != 0:
                    utils.publish_data_constant(self.pub_pos_enable, ['x', 'y'], False)

                # If any nonzero rpy power, disable those position pid loops
                elif self.powers['roll'] != 0 or self.powers['pitch'] != 0 or self.powers['yaw'] != 0:
                    utils.publish_data_constant(self.pub_pos_enable, ['roll', 'pitch', 'yaw'], False)

                # If any nonzero z power, disable those position pid loops
                if self.powers['z'] != 0:
                    utils.publish_data_constant(self.pub_pos_enable, ['z'], False)

                #TODO: BOTH cases

                utils.publish_data_dictionary(self.pub_power, self.DIRECTIONS, self.powers)
                # Publish current state to the desired state for PID
                utils.publish_data_dictionary(self.pub_pos, self.DIRECTIONS, self.hold)
                self.powers = None

if __name__ == '__main__':
    DesiredStateHandler().run()
