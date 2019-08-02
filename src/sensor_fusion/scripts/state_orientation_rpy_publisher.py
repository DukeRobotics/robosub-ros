#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion


class StateOrientationRpyPublisher(object):

    LISTENING_TOPIC = 'state'
    PUBLISHING_TOPIC_ROLL = 'state/roll'
    PUBLISHING_TOPIC_PITCH = 'state/pitch'
    PUBLISHING_TOPIC_YAW = 'state/yaw'

    def __init__(self):
        self._pub_roll = rospy.Publisher(self.PUBLISHING_TOPIC_ROLL, Float64)
        self._pub_pitch = rospy.Publisher(self.PUBLISHING_TOPIC_PITCH, Float64)
        self._pub_yaw = rospy.Publisher(self.PUBLISHING_TOPIC_YAW, Float64)

        rospy.Subscriber(self.LISTENING_TOPIC, Odometry, self._on_receive_state)

    def _on_receive_state(self, state):
        rpy = self.to_rpy(state.pose.pose.orientation)
        self._pub_roll.publish(rpy[0])
        self._pub_pitch.publish(rpy[1])
        self._pub_yaw.publish(rpy[2])

    def to_rpy(self, orientation):
        return euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])


if __name__ == '__main__':
    StateOrientationRpyPublisher()