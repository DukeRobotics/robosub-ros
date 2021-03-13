#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
import controls_utils as utils
from tf import TransformListener


class StateRepublisher:
    STATE_TOPIC = '/state'
    LOCAL_STATE_TOPIC = '/state_local'

    def __init__(self):
        rospy.init_node('state_republisher')
        self._pub_pose = {}
        self._pub_twist = {}
        self.listener = TransformListener()

        for d in utils.get_axes():
            self._pub_pose[d] = rospy.Publisher(utils.get_pose_topic(d), Float64, queue_size=3)
            self._pub_twist[d] = rospy.Publisher(utils.get_twist_topic(d), Float64, queue_size=3)

        self._pub_local_state = rospy.Publisher(self.LOCAL_STATE_TOPIC, Odometry, queue_size=3)

        rospy.Subscriber(self.STATE_TOPIC, Odometry, self.receive_odometry)
        rospy.spin()

    def receive_odometry(self, odometry):
        if 'base_link' in self.listener.getFrameStrings():
            local_pose = utils.transform_pose(self.listener, 'odom', 'base_link', odometry.pose.pose)

            utils.publish_data_dictionary(
                self._pub_pose,
                utils.get_axes(),
                utils.parse_pose(slocal_pose)
            )

            local_twist = utils.transform_twist(self.listener, 'odom', 'base_link', odometry.twist.twist)
            utils.publish_data_dictionary(
                self._pub_twist,
                utils.get_axes(),
                utils.parse_twist(local_twist)
            )

            local_state = Odometry()
            local_state.header.frame_id = 'base_link'
            local_state.pose.pose = local_pose
            local_state.twist.twist = local_twist
            self._pub_local_state.publish(local_state)


def main():
    try:
        StateRepublisher()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
