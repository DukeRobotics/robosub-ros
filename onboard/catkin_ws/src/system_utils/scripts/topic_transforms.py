#!/usr/bin/env python

import math

from transforms3d.euler import quat2euler

import rospy
from geometry_msgs.msg import Pose, Twist, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


# Class to store data for topic transformations
class TopicTransformData:
    def __init__(self, input_topic, input_type, input_type_conversion, output_topic, output_type,
                 output_type_conversion, subscriber=None, publisher=None, publisher_queue_size=1):
        self.input_topic = input_topic
        self.input_type = input_type
        self.input_type_conversion = input_type_conversion
        self.output_topic = output_topic
        self.output_type = output_type
        self.output_type_conversion = output_type_conversion
        self.subscriber = subscriber
        self.publisher = publisher
        self.publisher_queue_size = publisher_queue_size


# Class to store conversion functions
class Conversions:

    @staticmethod
    def quat_to_vector(quat_msg):
        # Convert quaternion to euler angles
        euler_angles = quat2euler(
            [quat_msg.w, quat_msg.x, quat_msg.y, quat_msg.z], axes='sxyz')

        # Convert to degrees
        euler_angles = [math.degrees(angle) for angle in euler_angles]

        vector_msg = Vector3()
        vector_msg.x = euler_angles[0]
        vector_msg.y = euler_angles[1]
        vector_msg.z = euler_angles[2]

        return vector_msg

    @staticmethod
    def pose_to_twist(pose_msg):
        # Create a Twist message with identical linear position
        twist_msg = Twist()
        twist_msg.linear.x = pose_msg.position.x
        twist_msg.linear.y = pose_msg.position.y
        twist_msg.linear.z = pose_msg.position.z

        twist_msg.angular = Conversions.quat_to_vector(pose_msg.orientation)

        return twist_msg


# Class to perform topic transformations
class TopicTransforms:

    # List of topic transformation data
    TOPIC_TRANSFORM_DATA = [
        TopicTransformData('/state', Odometry, lambda x: x.pose.pose, '/transforms/state/pose', Twist,
                           Conversions.pose_to_twist),
        TopicTransformData('/vectornav/IMU', Imu, lambda x: x.orientation, '/transforms/vectornav/IMU', Vector3,
                           Conversions.quat_to_vector),
        TopicTransformData('/controls/desired_pose', Pose, lambda x: x, '/transforms/controls/desired_pose', Twist,
                           Conversions.pose_to_twist),
    ]

    def __init__(self):
        rospy.init_node('topic_transforms')

        # Create subscribers and publishers for each topic transformation
        for data in self.TOPIC_TRANSFORM_DATA:
            data.subscriber = rospy.Subscriber(data.input_topic, data.input_type, self.callback, data)
            data.publisher = rospy.Publisher(data.output_topic, data.output_type, queue_size=data.publisher_queue_size)

    # Callback function to transform input message and publish output message
    def callback(self, msg, data):
        converted_input_type = data.input_type_conversion(msg)
        ouput_msg = data.output_type_conversion(converted_input_type)
        data.publisher.publish(ouput_msg)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    TopicTransforms().run()
