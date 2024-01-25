#!/usr/bin/env python

from transforms3d.euler import quat2euler

import rospy
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


class PoseTransformData:
    def __init__(self, input_topic, input_type, input_type_conversion, output_topic, output_type,
                 output_type_conversion):
        self.input_topic = input_topic
        self.input_type = input_type
        self.input_type_conversion = input_type_conversion
        self.output_topic = output_topic
        self.output_type = output_type
        self.output_type_conversion = output_type_conversion


def convert_pose_to_twist(pose_msg):
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


def convert_quat_to_vector(quat_msg):
    # Convert quaternion to euler angles
    euler_angles = quat2euler(
        [quat_msg.x, quat_msg.y, quat_msg.z, quat_msg.w])

    # Convert to degrees
    euler_angles = [angle * 180 / 3.14159265359 for angle in euler_angles]

    vector_msg = Vector3()
    vector_msg.x = euler_angles[0]
    vector_msg.y = euler_angles[1]
    vector_msg.z = euler_angles[2]

    return vector_msg


class PoseTransforms:

    POSE_TRANSFORM_DATA = [
        PoseTransformData('/state', Odometry, lambda x: x.pose.pose, '/state/pose_transformed', Twist,
                          convert_pose_to_twist),
        PoseTransformData('/vectornav/IMU', Imu, lambda x: x.orientation, '/vectornav/IMU/quat_transformed', Vector3,
                          convert_quat_to_vector)
    ]

    def __init__(self):
        rospy.init_node('pose_transforms')

        self.subscribers = {}
        self.publishers = {}

        for data in self.POSE_TRANSFORM_DATA:
            self.subscribers[data.input_topic] = rospy.Subscriber(data.input_topic, data.input_type, self.callback,
                                                                  data)
            self.publishers[data.output_topic] = rospy.Publisher(data.output_topic, data.output_type, queue_size=1)

    def callback(self, msg, data):
        converted_input_type = data.input_type_conversion(msg)
        ouput_msg = data.output_type_conversion(converted_input_type)
        self.publishers[data.output_topic].publish(ouput_msg)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    PoseTransforms().run()
