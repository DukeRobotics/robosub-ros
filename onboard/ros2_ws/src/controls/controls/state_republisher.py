#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
import controls.controls_utils as utils
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer


class StateRepublisher(Node):
    """Handles re-publishing of monolithic state topic (Odom) to separate topics. Also handles conversion from global
    to local reference frame (controls operates solely in the local frame). PID loops require individual state topics
    to compare to set-points.

    Attributes:
        listener: The ROS transformation listener used to transform from global to local reference frame
    """

    NODE_NAME = 'state_republisher'
    STATE_TOPIC = '/state'
    LOCAL_STATE_TOPIC = '/state_local'

    def __init__(self):
        """Initializes pub/sub config used to transform and isolate state topics."""
        super().__init__(self.NODE_NAME)
        self._pub_pose = {}
        self._pub_twist = {}
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        for d in utils.get_axes():
            self._pub_pose[d] = self.create_publisher(
                Float64, self._get_pose_topic(d), 3)
            self._pub_pose[d].publish(Float64(data=0.0))
            self._pub_twist[d] = self.create_publisher(
                Float64, self._get_twist_topic(d), 3)
            self._pub_twist[d].publish(Float64(data=0.0))

        self._pub_local_state = self.create_publisher(Odometry, self.LOCAL_STATE_TOPIC, 3)

        self.create_subscription(Odometry, self.STATE_TOPIC, self._receive_odometry, 10)

    def _get_pose_topic(self, axis):
        return '/controls/state/pose/' + axis

    def _get_twist_topic(self, axis):
        return '/controls/state/twist/' + axis

    def _receive_odometry(self, odometry):
        """Callback that handles re-publishing of state topics and transformation to local frame.

        Args:
            odometry: The Odom message holding the state of the robot in the global frame of reference
        """
        if self.tf_buffer.can_transform('odom', 'base_link', rclpy.time.Time()):
            transform = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
            local_pose = utils.transform_pose(odometry.pose.pose, transform)
            local_twist = utils.transform_twist(odometry.twist.twist, transform)

            utils.publish_data_dictionary(self._pub_pose, utils.parse_pose(local_pose))
            utils.publish_data_dictionary(self._pub_twist, utils.parse_twist(local_twist))

            local_state = Odometry()
            local_state.header.frame_id = 'base_link'
            local_state.pose.pose = local_pose  # This should always be 0
            local_state.twist.twist = local_twist
            self._pub_local_state.publish(local_state)


def main(args=None):
    try:
        rclpy.init(args=args)
        sr = StateRepublisher()
        rclpy.spin(sr)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        raise
    finally:
        sr.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
