#!/usr/bin/env python
import rospy
import numpy as np
import math

from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray
from tf.transformations import euler_from_quaternion

IMU_RAW_TOPIC = '/mavros/imu/data'
IMU_DEST_TOPIC_QUAT = 'sensors/imu/quat'
IMU_DEST_TOPIC_RPY = 'sensors/imu/rpy'
IMU_DEST_TOPIC_ROLL = 'sensors/imu/roll'
IMU_DEST_TOPIC_PITCH = 'sensors/imu/pitch'
IMU_DEST_TOPIC_YAW = 'sensors/imu/yaw'
NODE_NAME = 'imu_pub'

imu_pub_quat = rospy.Publisher(IMU_DEST_TOPIC_QUAT, Imu, queue_size=50)
imu_pub_rpy = rospy.Publisher(IMU_DEST_TOPIC_RPY, Float64MultiArray, queue_size=50)


def process_data(msg):
    msg.header.frame_id = "imu_link"

    imu_pub_quat.publish(msg)

    rpy = Float64MultiArray()
    rpy.data = to_rpy(msg.orientation)
    imu_pub_rpy.publish(rpy)

def to_rpy(orientation):
    return euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

def listener():
    rospy.init_node(NODE_NAME)
    rospy.Subscriber(IMU_RAW_TOPIC, Imu, process_data)
    rospy.spin()

if __name__ == '__main__':
    listener()
