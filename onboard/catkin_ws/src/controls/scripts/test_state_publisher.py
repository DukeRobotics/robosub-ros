#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
import controls_utils
import tf.transformations
from tf import TransformListener

class TestStatePublisher:
    PUBLISHING_TOPIC_DESIRED_POSE = 'controls/desired_pose'
    PUBLISHING_TOPIC_DESIRED_TWIST = 'controls/desired_twist'
    PUBLISHING_TOPIC_CURRENT_STATE = '/state'
    PUBLISHING_TOPIC_DESIRED_POWER = 'controls/desired_power'

    def __init__(self):
        rospy.init_node('test_state_publisher')
        self.listener = TransformListener()

        self._pub_desired_pose = rospy.Publisher(self.PUBLISHING_TOPIC_DESIRED_POSE, Pose, queue_size=3)
        self._pub_desired_twist = rospy.Publisher(self.PUBLISHING_TOPIC_DESIRED_TWIST, Twist, queue_size=3)
        self._pub_desired_power = rospy.Publisher(self.PUBLISHING_TOPIC_DESIRED_POWER, Twist, queue_size=3)
        self._pub_current_state = rospy.Publisher(self.PUBLISHING_TOPIC_CURRENT_STATE, Odometry, queue_size=3)

        # These values correspond to the desired global pose of the robot
        self.desired_pose_global = Pose()
        self.desired_pose_global.position.x = 1
        self.desired_pose_global.position.y = 0
        self.desired_pose_global.position.z = 0
        roll = 0
        pitch = 0
        yaw = 0

        q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        self.desired_pose_global.orientation.x = q[0]
        self.desired_pose_global.orientation.y = q[1]
        self.desired_pose_global.orientation.z = q[2]
        self.desired_pose_global.orientation.w = q[3]

        # These values correspond to the desired local pose of the robot
        self.desired_pose_local = Pose()
        self.desired_pose_local.position.x = 1
        self.desired_pose_local.position.y = 0
        self.desired_pose_local.position.z = 0
        roll = 0
        pitch = 0
        yaw = 0

        q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        self.desired_pose_local.orientation.x = q[0]
        self.desired_pose_local.orientation.y = q[1]
        self.desired_pose_local.orientation.z = q[2]
        self.desired_pose_local.orientation.w = q[3]
        self.desired_pose_transformed = controls_utils.transform_pose(self.listener, "base_link", "odom", self.desired_pose_local)

        # These values correspond to the desired global twist for the robot
        # Max linear z speed is ~ -0.26 -- ignore (for different mass)
        self.desired_twist_global = Twist()
        self.desired_twist_global.linear.x = 1
        self.desired_twist_global.linear.y = 0
        self.desired_twist_global.linear.z = 0
        self.desired_twist_global.angular.x = 0
        self.desired_twist_global.angular.y = 0
        self.desired_twist_global.angular.z = 0
        self.desired_twist_transformed = controls_utils.transform_twist(self.listener, "odom", "base_link", self.desired_twist_global)

        # These values correspond to the desired local twist for the robot
        self.desired_twist_local = Twist()
        self.desired_twist_local.linear.x = 1
        self.desired_twist_local.linear.y = 0
        self.desired_twist_local.linear.z = 0
        self.desired_twist_local.angular.x = 0
        self.desired_twist_local.angular.y = 0
        self.desired_twist_local.angular.z = 0

        # These values correspond to the desired twist for the robot
        self.desired_power = Twist()
        self.desired_power.linear.x = 1
        self.desired_power.linear.y = 0
        self.desired_power.linear.z = 0
        self.desired_power.angular.x = 0
        self.desired_power.angular.y = 0
        self.desired_power.angular.z = 0

        self.current_state = Odometry()
        self.current_state.pose.pose.position.x = -7
        self.current_state.pose.pose.position.y = 0
        self.current_state.pose.pose.position.z = 0
        self.current_state.pose.pose.orientation.x = 0
        self.current_state.pose.pose.orientation.y = 0
        self.current_state.pose.pose.orientation.w = 1

        self.current_state.twist.twist.linear.x = 0
        self.current_state.twist.twist.linear.y = 0
        self.current_state.twist.twist.linear.z = 0
        self.current_state.twist.twist.angular.x = 0
        self.current_state.twist.twist.angular.y = 0
        self.current_state.twist.twist.angular.z = 0

        self.current_state.header.frame_id = 'odom'
        self.current_state.header.stamp = rospy.Time()

    def publish_desired_pose_global(self):
        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            self._pub_desired_pose.publish(self.desired_pose_global)
            # self._pub_current_state.publish(self.current_state)
            rate.sleep()

    def publish_desired_pose_local(self):
        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            self._pub_desired_pose.publish(self.desired_pose_transformed)
            # self._pub_current_state.publish(self.current_state)
            rate.sleep()

    def publish_desired_twist_local(self):
        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            self._pub_desired_twist.publish(self.desired_twist_local)
            # self._pub_current_state.publish(self.current_state)
            rate.sleep()

    def publish_desired_twist_global(self):
        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            self._pub_desired_twist.publish(self.desired_twist_transformed)
            # self._pub_current_state.publish(self.current_state)
            rate.sleep()

    def publish_desired_power(self):
        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            self._pub_desired_power.publish(self.desired_power)
            # self._pub_current_state.publish(self.current_state)
            rate.sleep()


def main():
    # TestStatePublisher().publish_desired_pose_global()
    TestStatePublisher().publish_desired_pose_local()
    # TestStatePublisher().publish_desired_twist_local()
    # TestStatePublisher().publish_desired_twist_global()
    # TestStatePublisher().publish_desired_power()


if __name__ == '__main__':
    main()
