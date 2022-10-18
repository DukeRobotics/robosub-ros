#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
import controls_utils as utils
from tf import TransformListener


class StateRepublisher:
    """Handles re-publishing of monolithic state topic (Odom) to separate topics. Also handles conversion from global
    to local reference frame (controls operates solely in the local frame). PID loops require individual state topics
    to compare to set-points.

    Attributes:
        listener: The ROS transformation listener used to transform from global to local reference frame
    """

    STATE_TOPIC = '/state'
    LOCAL_STATE_TOPIC = '/state_local'

    def __init__(self):
        """Initializes pub/sub config used to transform and isolate state topics."""
        rospy.init_node('state_republisher')
        self._pub_pose = {}
        self._pub_twist = {}
        self.listener = TransformListener()

        for d in utils.get_axes():
            self._pub_pose[d] = rospy.Publisher(utils.get_pose_topic(d), Float64, latch=True, queue_size=3)
            self._pub_pose[d].publish(Float64(0))
            self._pub_twist[d] = rospy.Publisher(utils.get_twist_topic(d), Float64, latch=True, queue_size=3)
            self._pub_twist[d].publish(Float64(0))

        self._pub_local_state = rospy.Publisher(self.LOCAL_STATE_TOPIC, Odometry, queue_size=3)

        rospy.Subscriber(self.STATE_TOPIC, Odometry, self._receive_odometry)
        rospy.spin()

    def _get_pose_topic(self, axis):
        return '/controls/state/pose/' + axis

    def _get_twist_topic(self, axis):
        return '/controls/state/twist/' + axis

    def _receive_odometry(self, odometry):
        """Callback that handles re-publishing of state topics and transformation to local frame.

        Args:
            odometry: The Odom message holding the state of the robot in the global frame of reference
        """
        if 'base_link' in self.listener.getFrameStrings():
            local_pose = utils.transform_pose(self.listener, 'odom', 'base_link', odometry.pose.pose)
            utils.publish_data_dictionary(self._pub_pose, utils.parse_pose(local_pose))

            local_twist = utils.transform_twist(self.listener, 'odom', 'base_link', odometry.twist.twist)
            utils.publish_data_dictionary(self._pub_twist, utils.parse_twist(local_twist))

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
