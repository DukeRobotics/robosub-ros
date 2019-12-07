#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64, Header
from geometry_msgs.msg import PoseStamped, Pose

class TestStatePublisher():
    PUBLISHING_TOPIC_DESIRED_STATE = '/test/desired_pose'
    PUBLISHING_TOPIC_X = '/state/x'
    PUBLISHING_TOPIC_Y = '/state/y'
    PUBLISHING_TOPIC_Z = '/state/z'
    PUBLISHING_TOPIC_ROLL = '/state/roll'
    PUBLISHING_TOPIC_PITCH = '/state/pitch'
    PUBLISHING_TOPIC_YAW = '/state/yaw'

    def __init__(self):

        self._pub_desired_state = rospy.Publisher(self.PUBLISHING_TOPIC_DESIRED_STATE, PoseStamped, queue_size=3)

        self._pub_x = rospy.Publisher(self.PUBLISHING_TOPIC_X, Float64, queue_size=3)
        self._pub_y = rospy.Publisher(self.PUBLISHING_TOPIC_Y, Float64, queue_size=3)
        self._pub_z = rospy.Publisher(self.PUBLISHING_TOPIC_Z, Float64, queue_size=3)
        self._pub_roll = rospy.Publisher(self.PUBLISHING_TOPIC_ROLL, Float64, queue_size=3)
        self._pub_pitch = rospy.Publisher(self.PUBLISHING_TOPIC_PITCH, Float64, queue_size=3)
        self._pub_yaw = rospy.Publisher(self.PUBLISHING_TOPIC_YAW, Float64, queue_size=3)

        self.test_values = [0, 0, 0, 0, 0, 0]
        self.testPose = Pose()
        self.testPose.position.x = 1
        self.testPose.position.y = 0
        self.testPose.position.z = 0
        self.testPose.orientation.x = 0
        self.testPose.orientation.y = 0
        self.testPose.orientation.z = 0
        self.testPose.orientation.w = 1


        self.desired_state = PoseStamped()
        self.desired_state.pose = self.testPose

    def publish_to_state(self):
        rospy.init_node('test_state_publisher')
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            self._pub_desired_state.publish(self.desired_state)

            self._pub_x.publish(self.test_values[0])
            self._pub_y.publish(self.test_values[1])
            self._pub_z.publish(self.test_values[2])
            self._pub_roll.publish(self.test_values[3])
            self._pub_pitch.publish(self.test_values[4])
            self._pub_yaw.publish(self.test_values[5])



            rate.sleep()

def main():
    TestStatePublisher().publish_to_state()

if __name__ == '__main__':
    main()
