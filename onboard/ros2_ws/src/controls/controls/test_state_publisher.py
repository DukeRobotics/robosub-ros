#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
import controls.controls_utils as utils
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer


class TestStatePublisher(Node):

    NODE_NAME = 'test_state_publisher'
    DESIRED_POSE_TOPIC = 'controls/desired_pose'
    DESIRED_TWIST_TOPIC = 'controls/desired_twist'
    DESIRED_POWER_TOPIC = 'controls/desired_power'
    CURRENT_STATE_TOPIC = '/state'

    def __init__(self):
        super().__init__(self.NODE_NAME)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._pub_desired_pose = self.create_publisher(Pose, self.DESIRED_POSE_TOPIC, 3)
        self._pub_desired_twist = self.create_publisher(Twist, self.DESIRED_TWIST_TOPIC, 3)
        self._pub_desired_power = self.create_publisher(Twist, self.DESIRED_POWER_TOPIC, 3)
        self._pub_current_state = self.create_publisher(Odometry, self.CURRENT_STATE_TOPIC, 3)

        # These values correspond to the desired global pose of the robot
        self.desired_pose_global = Pose()
        self.desired_pose_global.position.x = 1.0
        self.desired_pose_global.position.y = 0.0
        self.desired_pose_global.position.z = 0.0
        self.desired_pose_global.orientation.x = 0.0
        self.desired_pose_global.orientation.y = 0.0
        self.desired_pose_global.orientation.z = 0.0
        self.desired_pose_global.orientation.w = 1.0

        # These values correspond to the desired local pose of the robot
        self.desired_pose_local = Pose()
        self.desired_pose_local.position.x = 0.0
        self.desired_pose_local.position.y = 0.0
        self.desired_pose_local.position.z = 0.0
        self.desired_pose_local.orientation.x = 0.0
        self.desired_pose_local.orientation.y = 0.0
        self.desired_pose_local.orientation.z = 0.0
        self.desired_pose_local.orientation.w = 1.0
        self.calculate_local_pose()

        # These values correspond to the desired local twist for the robot
        # Max linear z speed is ~ -0.26 -- ignore (for different mass)
        self.desired_twist = Twist()
        self.desired_twist.linear.x = 0.0
        self.desired_twist.linear.y = 0.0
        self.desired_twist.linear.z = 0.0
        self.desired_twist.angular.x = 0.0
        self.desired_twist.angular.y = 0.0
        self.desired_twist.angular.z = 1.0

        # These values correspond to the desired twist for the robot
        self.desired_power = Twist()
        self.desired_power.linear.x = 0.0
        self.desired_power.linear.y = 0.0
        self.desired_power.linear.z = 0.0
        self.desired_power.angular.x = 0.0
        self.desired_power.angular.y = 0.0
        self.desired_power.angular.z = 0.0

        self.current_state = Odometry()
        self.current_state.pose.pose.position.x = 0.0
        self.current_state.pose.pose.position.y = 0.0
        self.current_state.pose.pose.position.z = 0.0
        self.current_state.pose.pose.orientation.x = 0.0
        self.current_state.pose.pose.orientation.y = 0.0
        self.current_state.pose.pose.orientation.z = 0.0
        self.current_state.pose.pose.orientation.w = 1.0

        self.current_state.twist.twist.linear.x = 0.0
        self.current_state.twist.twist.linear.y = 0.0
        self.current_state.twist.twist.linear.z = 0.0
        self.current_state.twist.twist.angular.x = 0.0
        self.current_state.twist.twist.angular.y = 0.0
        self.current_state.twist.twist.angular.z = 0.0

        self.current_state.header.frame_id = 'odom'
        self.current_state.header.stamp = rclpy.time.Time()
        self.setup()

    def publish_current_state(self):
        self._pub_current_state.publish(self.current_state)

    def publish_desired_pose_global(self):
        self._pub_desired_pose.publish(self.desired_pose_global)

    def publish_desired_pose_local(self):
        self._pub_desired_pose.publish(self.desired_pose_transformed)

    def publish_desired_twist(self):
        self._pub_desired_twist.publish(self.desired_twist)

    def publish_desired_power(self):
        self._pub_desired_power.publish(self.desired_power)

    def calculate_local_pose(self):
        transform = self.tf_buffer.lookup_transform(
            'base_link', 'odom', rclpy.time.Time(), timeout=Duration(seconds=5.0))
        self.desired_pose_transformed = utils.transform_pose(self.desired_pose_local, transform)

    def setup(self):
        """ Un-comment which type of control to run """
        self.create_timer(1/self.PUBLISH_RATE, self.publish_desired_pose_global)
        # self.create_timer(1/self.PUBLISH_RATE, self.publish_desired_pose_local)
        # self.create_timer(1/self.PUBLISH_RATE, self.publish_desired_twist)
        # self.create_timer(1/self.PUBLISH_RATE, self.publish_desired_power)

        self.create_timer(1/self.PUBLISH_RATE, self.publish_current_state)


def main(args=None):
    try:
        rclpy.init(args=args)
        tsp = TestStatePublisher()
        rclpy.spin(tsp)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        raise
    finally:
        tsp.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
