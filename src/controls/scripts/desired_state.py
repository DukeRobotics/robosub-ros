#!/usr/bin/env python

import rospy
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import tf2_ros
import tf2_geometry_msgs
import numpy as np
from std_msgs.msg import Float64

from geometry_msgs.msg import Pose, Twist
# from nav_msgs.msg import Odometry


class DesiredStateHandler():

    DESIRED_LOCAL_TWIST_TOPIC = 'controls/desired_twist_local'
    DESIRED_GLOBAL_TWIST_TOPIC = 'controls/desired_twist_global'
    DESIRED_POSE_TOPIC = 'controls/desired_pose_global'
    PUBLISHING_TOPIC_X = 'global_x/setpoint'
    PUBLISHING_TOPIC_Y = 'global_y/setpoint'
    PUBLISHING_TOPIC_Z = 'global_z/setpoint'
    PUBLISHING_TOPIC_ROLL = 'global_roll/setpoint'
    PUBLISHING_TOPIC_PITCH = 'global_pitch/setpoint'
    PUBLISHING_TOPIC_YAW = 'global_yaw/setpoint' 

    pose = None
    local_twist = None
    global_twist = None

    def __init__(self):
        self._pub_x = rospy.Publisher(self.PUBLISHING_TOPIC_X, Float64, queue_size=3)
        self._pub_y = rospy.Publisher(self.PUBLISHING_TOPIC_Y, Float64, queue_size=3)
        self._pub_z = rospy.Publisher(self.PUBLISHING_TOPIC_Z, Float64, queue_size=3)
        self._pub_roll = rospy.Publisher(self.PUBLISHING_TOPIC_ROLL, Float64, queue_size=3)
        self._pub_pitch = rospy.Publisher(self.PUBLISHING_TOPIC_PITCH, Float64, queue_size=3)
        self._pub_yaw = rospy.Publisher(self.PUBLISHING_TOPIC_YAW, Float64, queue_size=3)

        rospy.Subscriber(self.DESIRED_POSE_TOPIC, Pose, self.receive_pose)
        #rospy.Subscriber(self.DESIRED_LOCAL_TWIST_TOPIC, Twist, self.receive_local_twist)
        #rospy.Subscriber(self.DESIRED_GLOBAL_TWIST_TOPIC, Twist, self.receive_global_twist)

    def receive_pose(self, pose):
        self.pose = pose

    def receive_local_twist(self, twist):
        self.local_twist = twist

    def receive_global_twist(self, twist):
        self.global_twist = twist

    def run(self):

        rospy.init_node('desired_state')
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            if not self.pose and not self.local_twist and not self.global_twist:
                # TODO: If it's been too long, stop PID and warn (timeout)
                continue
            if (self.pose and self.local_twist) or (self.pose and self.global_twist) or (self.local_twist and self.global_twist):
                # Make this if statement prettier
                # TODO: More than one seen in one update cycle, so warn and continue
                continue

            # Now we have either pose XOR local_twist XOR global_twist

            if self.pose:
                x = self.pose.position.x
                y = self.pose.position.y
                z = self.pose.position.z

                orientation = self.pose.orientation
                roll, pitch, yaw = euler_from_quaternion([orientation.x,
                                                          orientation.y,
                                                          orientation.z,
                                                          orientation.w])

                self._pub_x.publish(x)
                self._pub_y.publish(y)
                self._pub_z.publish(z)
                self._pub_roll.publish(roll)
                self._pub_pitch.publish(pitch)
                self._pub_yaw.publish(yaw)

                self.pose = None

            elif self.local_twist:
                # TODO: Perform necessary transformations from local to global frame and publish
                self.local_twist = None

            elif self.global_twist:
                # TODO: implement this, might be the same implementation as position
                self.global_twist = None

            rate.sleep()


def main():
    DesiredStateHandler().run()

if __name__ == '__main__':
    main()
