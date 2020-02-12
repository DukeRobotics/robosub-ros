#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry

class TestStatePublisher():
    PUBLISHING_TOPIC_DESIRED_STATE = 'controls/desired_pose_global'
    PUBLISHING_ODOMETRY_TOPIC = '/state'

    def __init__(self):

        self._pub_desired_state = rospy.Publisher(self.PUBLISHING_TOPIC_DESIRED_STATE, Pose, queue_size=3)

        #self._pub_odometry = rospy.Publisher(self.PUBLISHING_ODOMETRY_TOPIC, Odometry, queue_size=3)
        #These values correspond to the current state of the robot
        self.test_pose_values = [0, 0, 0, 0, 0, 0, 1]
        self.test_twist_values = [0, 0, 0, 0, 0, 0]

        self.desired_state = Pose()

        #These values correspond to the desired state of the robot
        self.desired_state.position.x = 2
        self.desired_state.position.y = 2
        self.desired_state.position.z = -2
        self.desired_state.orientation.x = 0
        self.desired_state.orientation.y = 0
        self.desired_state.orientation.z = 0
        self.desired_state.orientation.w = 1
        '''
        self.current_state = Odometry()
        self.current_state.pose.pose.position.x = self.test_pose_values[0]
        self.current_state.pose.pose.position.y = self.test_pose_values[1]
        self.current_state.pose.pose.position.z = self.test_pose_values[2]
        self.current_state.pose.pose.orientation.x = self.test_pose_values[3]
        self.current_state.pose.pose.orientation.y = self.test_pose_values[4]
        self.current_state.pose.pose.orientation.z = self.test_pose_values[5]
        self.current_state.pose.pose.orientation.w = self.test_pose_values[6]

        self.current_state.twist.twist.linear.x = self.test_twist_values[0]
        self.current_state.twist.twist.linear.y = self.test_twist_values[1]
        self.current_state.twist.twist.linear.z = self.test_twist_values[2]
        self.current_state.twist.twist.angular.x = self.test_twist_values[3]
        self.current_state.twist.twist.angular.y = self.test_twist_values[4]
        self.current_state.twist.twist.angular.z = self.test_twist_values[5]
        '''


    def publish_to_state(self):
        rospy.init_node('test_state_publisher')
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self._pub_desired_state.publish(self.desired_state)

            #self._pub_odometry.publish(self.current_state)

            rate.sleep()

def main():
    TestStatePublisher().publish_to_state()

if __name__ == '__main__':
    main()
