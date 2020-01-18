#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, TwistStamped, Quaternion
from nav_msgs.msg import Odometry

from drc_math import quad_vec_mult

class StateRepublisher():
    SIMULATION_POSE_TOPIC = '/sim/pose'
    SIMULATION_TWIST_TOPIC = '/sim/dvl'

    ROBOT_ODOMETRY_TOPIC = '/state'

    PUBLISHING_POSE_TOPIC_X = '/controls/state/pose/x'
    PUBLISHING_POSE_TOPIC_Y = '/controls/state/pose/y'
    PUBLISHING_POSE_TOPIC_Z = '/controls/state/pose/z'
    PUBLISHING_POSE_TOPIC_ROLL = '/controls/state/pose/roll'
    PUBLISHING_POSE_TOPIC_PITCH = '/controls/state/pose/pitch'
    PUBLISHING_POSE_TOPIC_YAW = '/controls/state/pose/yaw'

    PUBLISHING_TWIST_TOPIC_X = '/controls/state/twist/x'
    PUBLISHING_TWIST_TOPIC_Y = '/controls/state/twist/y'
    PUBLISHING_TWIST_TOPIC_Z = '/controls/state/twist/z'
    PUBLISHING_TWIST_TOPIC_ROLL = '/controls/state/twist/roll'
    PUBLISHING_TWIST_TOPIC_PITCH = '/controls/state/twist/pitch'
    PUBLISHING_TWIST_TOPIC_YAW = '/controls/state/twist/yaw'

    def __init__(self):

        self._pub_pose_x = rospy.Publisher(self.PUBLISHING_POSE_TOPIC_X, Float64, queue_size=3)
        self._pub_pose_y = rospy.Publisher(self.PUBLISHING_POSE_TOPIC_Y, Float64, queue_size=3)
        self._pub_pose_z = rospy.Publisher(self.PUBLISHING_POSE_TOPIC_Z, Float64, queue_size=3)
        self._pub_pose_roll = rospy.Publisher(self.PUBLISHING_POSE_TOPIC_ROLL, Float64, queue_size=3)
        self._pub_pose_pitch = rospy.Publisher(self.PUBLISHING_POSE_TOPIC_PITCH, Float64, queue_size=3)
        self._pub_pose_yaw = rospy.Publisher(self.PUBLISHING_POSE_TOPIC_YAW, Float64, queue_size=3)

        self._pub_twist_x = rospy.Publisher(self.PUBLISHING_TWIST_TOPIC_X, Float64, queue_size=3)
        self._pub_twist_y = rospy.Publisher(self.PUBLISHING_TWIST_TOPIC_Y, Float64, queue_size=3)
        self._pub_twist_z = rospy.Publisher(self.PUBLISHING_TWIST_TOPIC_Z, Float64, queue_size=3)
        self._pub_twist_roll = rospy.Publisher(self.PUBLISHING_TWIST_TOPIC_ROLL, Float64, queue_size=3)
        self._pub_twist_pitch = rospy.Publisher(self.PUBLISHING_TWIST_TOPIC_PITCH, Float64, queue_size=3)
        self._pub_twist_yaw = rospy.Publisher(self.PUBLISHING_TWIST_TOPIC_YAW, Float64, queue_size=3)

        rospy.init_node('state_republisher')

        self.mode = rospy.get_param('~mode', 'robot')  # robot or sim, default to robot
        #if self.mode == 'sim':
           # rospy.Subscriber(self.SIMULATION_POSE_TOPIC, PoseStamped, self.receive_pose_stamped)
          #  rospy.Subscriber(self.SIMULATION_TWIST_TOPIC, TwistStamped, self.receive_twist_stamped)
        rospy.Subscriber(self.ROBOT_ODOMETRY_TOPIC, Odometry, self.receive_odometry)

        rospy.spin()


    def receive_pose_stamped(self, pose_stamped):
        # xyz Position
        self._pub_pose_x.publish(pose_stamped.pose.position.x)
        self._pub_pose_y.publish(pose_stamped.pose.position.y)
        self._pub_pose_z.publish(pose_stamped.pose.position.z)
        # rpy Orientation
        vec = quad_vec_mult([pose_stamped.pose.orientation.x,
                    pose_stamped.pose.orientation.y,
                    pose_stamped.pose.orientation.z,
                    pose_stamped.pose.orientation.w], [1, 0, 0])
        self._pub_pose_roll.publish(vec[0])
        self._pub_pose_pitch.publish(vec[1])
        self._pub_pose_yaw.publish(vec[2])


    def receive_twist_stamped(self, twist_stamped):
        # Linear Velocity
        self._pub_twist_x.publish(pose_stamped.twist.linear.x)
        self._pub_twist_y.publish(pose_stamped.twist.linear.y)
        self._pub_twist_z.publish(pose_stamped.twist.linear.z)
        # Angular Velocity
        self._pub_twist_roll.publish(pose_stamped.twist.angular.roll)
        self._pub_twist_pitch.publish(pose_stamped.twist.angular.pitch)
        self._pub_twist_yaw.publish(pose_stamped.twist.angular.yaw)

    def receive_odometry(self, odometry):
        # xyz Position
        self._pub_pose_x.publish(odometry.pose.pose.position.x)
        self._pub_pose_y.publish(odometry.pose.pose.position.y)
        self._pub_pose_z.publish(odometry.pose.pose.position.z)
        # rpy Orientation
        vec = quad_vec_mult([odometry.pose.pose.orientation.x,
                            odometry.pose.pose.orientation.y,
                            odometry.pose.pose.orientation.z,
                            odometry.pose.pose.orientation.w], [1, 0, 0])
        self._pub_pose_roll.publish(vec[0])
        self._pub_pose_pitch.publish(vec[1])
        self._pub_pose_yaw.publish(vec[2])

        # Linear Velocity
        self._pub_twist_x.publish(odometry.twist.twist.linear.x)
        self._pub_twist_y.publish(odometry.twist.twist.linear.y)
        self._pub_twist_z.publish(odometry.twist.twist.linear.z)
        # Angular Velocity
        self._pub_twist_roll.publish(odometry.twist.twist.angular.x)
        self._pub_twist_pitch.publish(odometry.twist.twist.angular.y)
        self._pub_twist_yaw.publish(odometry.twist.twist.angular.z)


def main():
    StateRepublisher()

if __name__ == '__main__':
    main()
