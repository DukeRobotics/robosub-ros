#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion


class PositionRepublisher(object):

    LISTENING_TOPIC = 'state'
    PUBLISHING_TOPIC_X = 'local_state/x'
    PUBLISHING_TOPIC_Y = 'local_state/y'
    PUBLISHING_TOPIC_Z = 'local_state/z'

    def __init__(self):
        self._pub_x = rospy.Publisher(self.PUBLISHING_TOPIC_X, Float64, queue_size=10)
        self._pub_y = rospy.Publisher(self.PUBLISHING_TOPIC_Y, Float64, queue_size=10)
        self._pub_z = rospy.Publisher(self.PUBLISHING_TOPIC_Z, Float64, queue_size=10)

        rospy.Subscriber(self.LISTENING_TOPIC, Odometry, self._on_receive_state)

    def _on_receive_state(self, state):
        self._pub_x.publish(0)
        self._pub_y.publish(0)
        self._pub_z.publish(0)

    def run(self):
        rospy.init_node('state_rpy_publisher')
        rospy.spin()


if __name__ == '__main__':
    PositionRepublisher().run()
