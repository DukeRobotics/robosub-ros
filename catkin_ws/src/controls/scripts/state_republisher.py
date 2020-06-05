#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
import drc_utils as utils


class StateRepublisher():
    STATE_TOPIC = '/state'

    def __init__(self):
        self._pub_pose = {}
        self._pub_twist = {}
        for d in utils.get_directions():
            self._pub_pose[d] = rospy.Publisher(utils.get_pose_topic(d), Float64, queue_size=3)
            self._pub_twist[d] = rospy.Publisher(utils.get_twist_topic(d), Float64, queue_size=3)

        rospy.init_node('state_republisher')
        rospy.Subscriber(self.STATE_TOPIC, Odometry, self.receive_odometry)
        rospy.spin()

    def receive_odometry(self, odometry):
        pose = utils.parse_pose(odometry.pose.pose)
        utils.publish_data_dictionary(self._pub_pose, utils.get_directions(), pose)

        twist = utils.parse_twist(odometry.twist.twist)
        utils.publish_data_dictionary(self._pub_twist, utils.get_directions(), twist)


def main():
    try:
        StateRepublisher()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
