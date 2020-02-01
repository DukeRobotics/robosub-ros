#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose, Twist

class TestStatePublisher():
    PUBLISHING_TOPIC_DESIRED_POSE = 'controls/desired_pose_global'
    PUBLISHING_TOPIC_DESIRED_LOCAL_TWIST = 'controls/desired_twist_local'
    PUBLISHING_TOPIC_DESIRED_GLOBAL_TWIST = 'controls/desired_twist_global'

    def __init__(self):

        self._pub_desired_pose = rospy.Publisher(self.PUBLISHING_TOPIC_DESIRED_POSE, Pose, queue_size=3)
        self._pub_desired_local_twist = rospy.Publisher(self.PUBLISHING_TOPIC_DESIRED_LOCAL_TWIST, Twist, queue_size=3)
        self._pub_desired_global_twist = rospy.Publisher(self.PUBLISHING_TOPIC_DESIRED_GLOBAL_TWIST, Twist, queue_size=3)


        #These values correspond to the desired pose of the robot
        self.desired_pose = Pose()
        self.desired_pose.position.x = 2
        self.desired_pose.position.y = 2
        self.desired_pose.position.z = 0
        self.desired_pose.orientation.x = 0
        self.desired_pose.orientation.y = 0
        self.desired_pose.orientation.z = 0
        self.desired_pose.orientation.w = 1

        #These values correspond to the desired local twist of the robot
        self.desired_local_twist = Twist()
        self.desired_local_twist.linear.x = 1
        self.desired_local_twist.linear.y = 0
        self.desired_local_twist.linear.z = 0
        self.desired_local_twist.angular.x = 0
        self.desired_local_twist.angular.y = 0
        self.desired_local_twist.angular.z = 0


        #These values correspond to the desired global twist of the robot
        self.desired_global_twist = Twist()
        self.desired_global_twist.linear.x = 0
        self.desired_global_twist.linear.y = 1
        self.desired_global_twist.linear.z = 0
        self.desired_global_twist.angular.x = 0
        self.desired_global_twist.angular.y = 0
        self.desired_global_twist.angular.z = 0


    def publish_desired_pose(self):
        rospy.init_node('test_state_publisher')
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self._pub_desired_pose.publish(self.desired_pose)
            rate.sleep()

    def publish_desired_local_twist(self):
        rospy.init_node('test_state_publisher')
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self._pub_desired_local_twist.publish(self.desired_local_twist)
            rate.sleep()

    def publish_desired_global_twist(self):
        rospy.init_node('test_state_publisher')
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self._pub_desired_global_twist.publish(self.desired_global_twist)
            rate.sleep()




def main():
    #TestStatePublisher().publish_desired_pose()
    #TestStatePublisher().publish_desired_local_twist()
    TestStatePublisher().publish_desired_global_twist()

if __name__ == '__main__':
    main()
