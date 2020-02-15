#!/usr/bin/env python

import rospy
from tf.transformations import quaternion_from_euler, euler_from_quaternion, euler_matrix
import tf2_ros
import tf2_geometry_msgs
import numpy as np
from std_msgs.msg import Float64, Bool
from drc_math import RpToTrans, Adjoint

import time

from geometry_msgs.msg import Pose, Twist
# from nav_msgs.msg import Odometry

class bcolors:
    BOLD = '\033[1m'
    OKGREEN = '\033[92m'
    WARN = '\033[93m'
    FAIL = '\033[91m'
    RESET = '\033[0m'

class DesiredStateHandler():

    POSE_TOPIC_X = '/controls/state/pose/x'
    POSE_TOPIC_Y = '/controls/state/pose/y'
    POSE_TOPIC_Z = '/controls/state/pose/z'
    POSE_TOPIC_ROLL = '/controls/state/pose/roll'
    POSE_TOPIC_PITCH = '/controls/state/pose/pitch'
    POSE_TOPIC_YAW = '/controls/state/pose/yaw'

    DESIRED_TWIST_POWER = 'controls/desired_twist_power'
    DESIRED_POSE_TOPIC = 'controls/desired_pose'

    PUBLISHING_TOPIC_X_POS = 'controls/x_pos/setpoint'
    PUBLISHING_TOPIC_Y_POS = 'controls/y_pos/setpoint'
    PUBLISHING_TOPIC_Z_POS = 'controls/z_pos/setpoint'
    PUBLISHING_TOPIC_ROLL_POS = 'controls/roll_pos/setpoint'
    PUBLISHING_TOPIC_PITCH_POS = 'controls/pitch_pos/setpoint'
    PUBLISHING_TOPIC_YAW_POS = 'controls/yaw_pos/setpoint'

    PUBLISHING_TOPIC_ENABLE_X_POS = 'controls/enable/x_pos'
    PUBLISHING_TOPIC_ENABLE_Y_POS = 'controls/enable/y_pos'
    PUBLISHING_TOPIC_ENABLE_Z_POS = 'controls/enable/z_pos'
    PUBLISHING_TOPIC_ENABLE_ROLL_POS = 'controls/enable/roll_pos'
    PUBLISHING_TOPIC_ENABLE_PITCH_POS = 'controls/enable/pitch_pos'
    PUBLISHING_TOPIC_ENABLE_YAW_POS = 'controls/enable/yaw_pos'

    PUBLISHING_TOPIC_X_EFFORT = '/control_effort/x'
    PUBLISHING_TOPIC_Y_EFFORT = '/control_effort/y'
    PUBLISHING_TOPIC_Z_EFFORT = '/control_effort/z'
    PUBLISHING_TOPIC_ROLL_EFFORT = '/control_effort/roll'
    PUBLISHING_TOPIC_PITCH_EFFORT = '/control_effort/pitch'
    PUBLISHING_TOPIC_YAW_EFFORT = '/control_effort/yaw'

    REFRESH_HZ = 10  # for main loop

    x_hold = 0
    y_hold = 0
    z_hold = 0
    roll_hold = 0
    pitch_hold = 0
    yaw_hold = 0

    x = 0
    y = 0
    z = 0
    roll = 0
    pitch = 0
    yaw = 0

    pose = None
    powers = None
    last_powers = None

    def __init__(self):
        rospy.Subscriber(self.POSE_TOPIC_X, Float64, self.receive_x)
        rospy.Subscriber(self.POSE_TOPIC_Y, Float64, self.receive_y)
        rospy.Subscriber(self.POSE_TOPIC_Z, Float64, self.receive_z)
        rospy.Subscriber(self.POSE_TOPIC_ROLL, Float64, self.receive_roll)
        rospy.Subscriber(self.POSE_TOPIC_PITCH, Float64, self.receive_pitch)
        rospy.Subscriber(self.POSE_TOPIC_YAW, Float64, self.receive_yaw)

        #PID Position publishers
        self._pub_x_pos = rospy.Publisher(self.PUBLISHING_TOPIC_X_POS, Float64, queue_size=3)
        self._pub_y_pos = rospy.Publisher(self.PUBLISHING_TOPIC_Y_POS, Float64, queue_size=3)
        self._pub_z_pos = rospy.Publisher(self.PUBLISHING_TOPIC_Z_POS, Float64, queue_size=3)
        self._pub_roll_pos = rospy.Publisher(self.PUBLISHING_TOPIC_ROLL_POS, Float64, queue_size=3)
        self._pub_pitch_pos = rospy.Publisher(self.PUBLISHING_TOPIC_PITCH_POS, Float64, queue_size=3)
        self._pub_yaw_pos = rospy.Publisher(self.PUBLISHING_TOPIC_YAW_POS, Float64, queue_size=3)

        self._pub_x_pos_enable = rospy.Publisher(self.PUBLISHING_TOPIC_ENABLE_X_POS, Bool, queue_size=3)
        self._pub_y_pos_enable = rospy.Publisher(self.PUBLISHING_TOPIC_ENABLE_Y_POS, Bool, queue_size=3)
        self._pub_z_pos_enable = rospy.Publisher(self.PUBLISHING_TOPIC_ENABLE_Z_POS, Bool, queue_size=3)
        self._pub_roll_pos_enable = rospy.Publisher(self.PUBLISHING_TOPIC_ENABLE_ROLL_POS, Bool, queue_size=3)
        self._pub_pitch_pos_enable = rospy.Publisher(self.PUBLISHING_TOPIC_ENABLE_PITCH_POS, Bool, queue_size=3)
        self._pub_yaw_pos_enable = rospy.Publisher(self.PUBLISHING_TOPIC_ENABLE_YAW_POS, Bool, queue_size=3)

        self._pub_x_effort = rospy.Publisher(self.PUBLISHING_TOPIC_X_EFFORT, Float64, queue_size=3)
        self._pub_y_effort = rospy.Publisher(self.PUBLISHING_TOPIC_Y_EFFORT, Float64, queue_size=3)
        self._pub_z_effort = rospy.Publisher(self.PUBLISHING_TOPIC_Z_EFFORT, Float64, queue_size=3)
        self._pub_roll_effort = rospy.Publisher(self.PUBLISHING_TOPIC_ROLL_EFFORT, Float64, queue_size=3)
        self._pub_pitch_effort= rospy.Publisher(self.PUBLISHING_TOPIC_PITCH_EFFORT, Float64, queue_size=3)
        self._pub_yaw_effort = rospy.Publisher(self.PUBLISHING_TOPIC_YAW_EFFORT, Float64, queue_size=3)

        rospy.Subscriber(self.DESIRED_POSE_TOPIC, Pose, self.receive_pose)
        rospy.Subscriber(self.DESIRED_TWIST_POWER, Twist, self.receive_powers)

    def receive_x(self, x):
        self.x = x.data

    def receive_y(self, y):
        self.y = y.data

    def receive_z(self, z):
        self.z = z.data

    def receive_roll(self, roll):
        self.roll = roll.data

    def receive_pitch(self, pitch):
        self.pitch = pitch.data

    def receive_yaw(self, yaw):
        self.yaw = yaw.data

    def receive_pose(self, pose):
        self.pose = pose

    def receive_powers(self, twist):
        self.powers = twist

    def soft_estop(self):
        #Stop Moving
        self._pub_x_pos_enable.publish(False)
        self._pub_y_pos_enable.publish(False)
        self._pub_z_pos_enable.publish(False)
        self._pub_roll_pos_enable.publish(False)
        self._pub_pitch_pos_enable.publish(False)
        self._pub_yaw_pos_enable.publish(False)

        self._pub_x_effort.publish(0)
        self._pub_y_effort.publish(0)
        self._pub_z_effort.publish(0)
        self._pub_roll_effort.publish(0)
        self._pub_pitch_effort.publish(0)
        self._pub_yaw_effort.publish(0)

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
                self._pub_x_pos_enable.publish(True)
                self._pub_y_pos_enable.publish(True)
                self._pub_z_pos_enable.publish(True)
                self._pub_roll_pos_enable.publish(True)
                self._pub_pitch_pos_enable.publish(True)
                self._pub_yaw_pos_enable.publish(True)

                x = self.pose.position.x
                y = self.pose.position.y
                z = self.pose.position.z

                orientation = self.pose.orientation
                roll, pitch, yaw = euler_from_quaternion([orientation.x,
                                                          orientation.y,
                                                          orientation.z,
                                                          orientation.w])

                self._pub_x_pos.publish(x)
                self._pub_y_pos.publish(y)
                self._pub_z_pos.publish(z)
                self._pub_roll_pos.publish(roll)
                self._pub_pitch_pos.publish(pitch)
                self._pub_yaw_pos.publish(yaw)

                self.pose = None

            elif self.powers:
                if self.powers != self.last_powers:
                    # Enable all PID Loops
                    self._pub_x_pos_enable.publish(True)
                    self._pub_y_pos_enable.publish(True)
                    self._pub_z_pos_enable.publish(True)
                    self._pub_roll_pos_enable.publish(True)
                    self._pub_pitch_pos_enable.publish(True)
                    self._pub_yaw_pos_enable.publish(True)

                    # Hold Position on the current state
                    self.x_hold = self.x
                    self.y_hold = self.y
                    self.z_hold = self.z
                    self.roll_hold = self.roll
                    self.pitch_hold = self.pitch
                    self.yaw_hold = self.yaw

                    self.last_powers = self.powers

                # Nonzero entries bypass PID

                # If any nonzero xyz power, publish those powers directly
                if self.powers.linear.x != 0 or self.powers.linear.y != 0:
                    self._pub_x_pos_enable.publish(False)
                    self._pub_x_effort.publish(self.powers.linear.x)
                    self._pub_y_pos_enable.publish(False)
                    self._pub_y_effort.publish(self.powers.linear.y)

                # If any nonzero rpy power, publish those powers directly
                elif self.powers.angular.x != 0 or self.powers.angular.y != 0 or self.powers.angular.z != 0:
                    self._pub_roll_pos_enable.publish(False)
                    self._pub_roll_effort.publish(self.powers.angular.x)
                    self._pub_pitch_pos_enable.publish(False)
                    self._pub_pitch_effort.publish(self.powers.angular.y)
                    self._pub_yaw_pos_enable.publish(False)
                    self._pub_yaw_effort.publish(self.powers.angular.z)

                if self.powers.linear.z !=0:
                    self._pub_z_pos_enable.publish(False)
                    self._pub_z_effort.publish(self.powers.linear.z)

                #TODO: BOTH cases

                #Publish current state to the desired state for PID
                self._pub_x_pos.publish(self.x_hold)
                self._pub_y_pos.publish(self.y_hold)
                self._pub_z_pos.publish(self.z_hold)
                self._pub_roll_pos.publish(self.roll_hold)
                self._pub_pitch_pos.publish(self.pitch_hold)
                self._pub_yaw_pos.publish(self.yaw_hold)

                self.powers = None


def main():
    DesiredStateHandler().run()

if __name__ == '__main__':
    main()
