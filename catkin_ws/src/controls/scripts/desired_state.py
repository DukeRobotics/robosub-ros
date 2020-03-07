#!/usr/bin/env python

import rospy
from tf.transformations import euler_from_quaternion
#import tf2_ros
#import tf2_geometry_msgs

from std_msgs.msg import Float64, Bool
import time
from geometry_msgs.msg import Pose, Twist, Vector3

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

    PID_TOPIC_X = 'controls/x_pos/setpoint'
    PID_TOPIC_Y = 'controls/y_pos/setpoint'
    PID_TOPIC_Z = 'controls/z_pos/setpoint'
    PID_TOPIC_ROLL = 'controls/roll_pos/setpoint'
    PID_TOPIC_PITCH = 'controls/pitch_pos/setpoint'
    PID_TOPIC_YAW = 'controls/yaw_pos/setpoint'

    PID_ENABLE_X = 'controls/enable/x_pos'
    PID_ENABLE_Y = 'controls/enable/y_pos'
    PID_ENABLE_Z = 'controls/enable/z_pos'
    PID_ENABLE_ROLL = 'controls/enable/roll_pos'
    PID_ENABLE_PITCH = 'controls/enable/pitch_pos'
    PID_ENABLE_YAW = 'controls/enable/yaw_pos'

    POWER_TOPIC_X = '/controls/power/x'
    POWER_TOPIC_Y = '/controls/power/y'
    POWER_TOPIC_Z = '/controls/power/z'
    POWER_TOPIC_ROLL = '/controls/power/roll'
    POWER_TOPIC_PITCH = '/controls/power/pitch'
    POWER_TOPIC_YAW = '/controls/power/yaw'

    REFRESH_HZ = 10  # for main loop

    x_hold, y_hold, z_hold, roll_hold, pitch_hold, yaw_hold = 0, 0, 0, 0, 0, 0
    x, y, z, roll, pitch, yaw = 0, 0, 0, 0, 0, 0

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
        self._pub_x_pos = rospy.Publisher(self.PID_TOPIC_X, Float64, queue_size=3)
        self._pub_y_pos = rospy.Publisher(self.PID_TOPIC_Y, Float64, queue_size=3)
        self._pub_z_pos = rospy.Publisher(self.PID_TOPIC_Z, Float64, queue_size=3)
        self._pub_roll_pos = rospy.Publisher(self.PID_TOPIC_ROLL, Float64, queue_size=3)
        self._pub_pitch_pos = rospy.Publisher(self.PID_TOPIC_PITCH, Float64, queue_size=3)
        self._pub_yaw_pos = rospy.Publisher(self.PID_TOPIC_YAW, Float64, queue_size=3)

        self._pub_x_pos_enable = rospy.Publisher(self.PID_ENABLE_X, Bool, queue_size=3)
        self._pub_y_pos_enable = rospy.Publisher(self.PID_ENABLE_Y, Bool, queue_size=3)
        self._pub_z_pos_enable = rospy.Publisher(self.PID_ENABLE_Z, Bool, queue_size=3)
        self._pub_roll_pos_enable = rospy.Publisher(self.PID_ENABLE_ROLL, Bool, queue_size=3)
        self._pub_pitch_pos_enable = rospy.Publisher(self.PID_ENABLE_PITCH, Bool, queue_size=3)
        self._pub_yaw_pos_enable = rospy.Publisher(self.PID_ENABLE_YAW, Bool, queue_size=3)

        self._pub_x_power = rospy.Publisher(self.POWER_TOPIC_X, Float64, queue_size=3)
        self._pub_y_power = rospy.Publisher(self.POWER_TOPIC_Y, Float64, queue_size=3)
        self._pub_z_power = rospy.Publisher(self.POWER_TOPIC_Z, Float64, queue_size=3)
        self._pub_roll_power = rospy.Publisher(self.POWER_TOPIC_ROLL, Float64, queue_size=3)
        self._pub_pitch_power= rospy.Publisher(self.POWER_TOPIC_PITCH, Float64, queue_size=3)
        self._pub_yaw_power = rospy.Publisher(self.POWER_TOPIC_YAW, Float64, queue_size=3)

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

        self._pub_x_power.publish(0)
        self._pub_y_power.publish(0)
        self._pub_z_power.publish(0)
        self._pub_roll_power.publish(0)
        self._pub_pitch_power.publish(0)
        self._pub_yaw_power.publish(0)

        self.powers = None
        self.last_powers = None
        self.pose = None

    def twists_equal(self, t1, t2):
        return (t1.linear.x == t2.linear.x and
                t1.linear.y == t2.linear.y and
                t1.linear.z == t2.linear.z and
                t1.angular.x == t2.angular.x and
                t1.angular.y == t2.angular.y and
                t1.angular.z == t2.angular.z)

    def copy_twist(self, t):
        return Twist(Vector3(t.linear.x, t.linear.y, t.linear.z),
                     Vector3(t.angular.x, t.angular.y, t.angular.z))

    def enable_loops(self):
        #Enable all PID Loops
        self._pub_x_pos_enable.publish(True)
        self._pub_y_pos_enable.publish(True)
        self._pub_z_pos_enable.publish(True)
        self._pub_roll_pos_enable.publish(True)
        self._pub_pitch_pos_enable.publish(True)
        self._pub_yaw_pos_enable.publish(True)

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
                if self.last_powers is None or not self.twists_equal(self.powers, self.last_powers):
                    self.enable_loops()

                    # Hold Position on the current state
                    self.x_hold = self.x
                    self.y_hold = self.y
                    self.z_hold = self.z
                    self.roll_hold = self.roll
                    self.pitch_hold = self.pitch
                    self.yaw_hold = self.yaw

                    self.last_powers = self.copy_twist(self.powers)

                # Nonzero entries bypass PID

                # If any nonzero xy power, disable those position pid loops
                if self.powers.linear.x != 0 or self.powers.linear.y != 0:
                    self._pub_x_pos_enable.publish(False)
                    self._pub_y_pos_enable.publish(False)

                # If any nonzero rpy power, disable those position pid loops
                elif self.powers.angular.x != 0 or self.powers.angular.y != 0 or self.powers.angular.z != 0:
                    self._pub_roll_pos_enable.publish(False)
                    self._pub_pitch_pos_enable.publish(False)
                    self._pub_yaw_pos_enable.publish(False)

                # If any nonzero z power, disable those position pid loops
                if self.powers.linear.z !=0:
                    self._pub_z_pos_enable.publish(False)

                #TODO: BOTH cases

                self._pub_x_power.publish(self.powers.linear.x)
                self._pub_y_power.publish(self.powers.linear.y)
                self._pub_z_power.publish(self.powers.linear.z)
                self._pub_roll_power.publish(self.powers.angular.x)
                self._pub_pitch_power.publish(self.powers.angular.y)
                self._pub_yaw_power.publish(self.powers.angular.z)

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
