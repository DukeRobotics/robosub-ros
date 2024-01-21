#!/usr/bin/env python

from transforms3d.euler import quat2euler

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class PoseTransformData:
    def __init__(self, input_topic, input_type, input_type_to_pose_function, output_topic):
        self.input_topic = input_topic
        self.input_type = input_type
        self.input_type_to_pose_function = input_type_to_pose_function
        self.output_topic = output_topic


class PoseTransforms:

    POSE_TRANSFORM_DATA = [
        PoseTransformData('/state', Odometry, lambda x: x.pose.pose, '/state/pose_transformed')
    ]

    def __init__(self):
        rospy.init_node('pose_transforms')

        self.subscribers = {}
        self.publishers = {}

        for data in self.POSE_TRANSFORM_DATA:
            self.subscribers[data.input_topic] = rospy.Subscriber(data.input_topic, data.input_type, self.callback,
                                                                  data)
            self.publishers[data.output_topic] = rospy.Publisher(data.output_topic, Twist, queue_size=1)

    def callback(self, msg, data):
        pose = data.input_type_to_pose_function(msg)
        twist = self.convert_pose_to_twist(pose)
        self.publishers[data.output_topic].publish(twist)

    def convert_pose_to_twist(self, pose_msg):
        # Create a Twist message with identical linear position
        twist_msg = Twist()
        twist_msg.linear.x = pose_msg.position.x
        twist_msg.linear.y = pose_msg.position.y
        twist_msg.linear.z = pose_msg.position.z

        # Convert quaternion to euler angles
        euler_angles = quat2euler(
            [pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w])

        # Convert to degrees
        euler_angles = [angle * 180 / 3.14159265359 for angle in euler_angles]

        # Set angular position
        twist_msg.angular.x = euler_angles[0]
        twist_msg.angular.y = euler_angles[1]
        twist_msg.angular.z = euler_angles[2]

        return twist_msg

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    PoseTransforms().run()
