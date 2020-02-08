#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry

class TestStatePublisher():
    PUBLISHING_TOPIC_DESIRED_POSE = 'controls/desired_pose'
    PUBLISHING_TOPIC_DESIRED_POWERS = 'controls/desired_twist_power'
    PUBLISHING_TOPIC_CURRENT_STATE = '/state'

    def __init__(self):

        self._pub_desired_pose = rospy.Publisher(self.PUBLISHING_TOPIC_DESIRED_POSE, Pose, queue_size=3)
        self._pub_desired_powers = rospy.Publisher(self.PUBLISHING_TOPIC_DESIRED_POWERS, Twist, queue_size=3)
        self._pub_current_state = rospy.Publisher(self.PUBLISHING_TOPIC_CURRENT_STATE, Odometry, queue_size=3)

        #These values correspond to the desired pose of the robot
        self.desired_pose = Pose()
        self.desired_pose.position.x = 2
        self.desired_pose.position.y = 2
        self.desired_pose.position.z = 0
        self.desired_pose.orientation.x = 0
        self.desired_pose.orientation.y = 0
        self.desired_pose.orientation.z = 0
        self.desired_pose.orientation.w = 1

        #These values correspond to the desired powers for the robot
        self.desired_powers = Twist()
        self.desired_powers.linear.x = 1
        self.desired_powers.linear.y = 0
        self.desired_powers.linear.z = 0
        self.desired_powers.angular.x = 0
        self.desired_powers.angular.y = 0
        self.desired_powers.angular.z = 0

        self.current_state = Odometry()
        self.current_state.pose.pose.position.x = 0
        self.current_state.pose.pose.position.y = 0
        self.current_state.pose.pose.position.z = 0
        self.current_state.pose.pose.orientation.x = 0
        self.current_state.pose.pose.orientation.y = 0
        self.current_state.pose.pose.orientation.z = 0
        self.current_state.pose.pose.orientation.w = 1

        self.current_state.twist.twist.linear.x = 0
        self.current_state.twist.twist.linear.y = 0
        self.current_state.twist.twist.linear.z = 0
        self.current_state.twist.twist.angular.x = 0
        self.current_state.twist.twist.angular.y = 0
        self.current_state.twist.twist.angular.z = 0

    def publish_desired_pose(self):
        rospy.init_node('test_state_publisher')
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self._pub_desired_pose.publish(self.desired_pose)
            #self._pub_current_state.publish(self.current_state)
            rate.sleep()

    def publish_desired_powers(self):
        rospy.init_node('test_state_publisher')
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self._pub_desired_powers.publish(self.desired_powers)
            #self._pub_current_state.publish(self.current_state)
            rate.sleep()


def main():
    #TestStatePublisher().publish_desired_pose()
    TestStatePublisher().publish_desired_powers()

if __name__ == '__main__':
    main()
