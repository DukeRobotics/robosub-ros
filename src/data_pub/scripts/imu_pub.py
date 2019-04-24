#!/usr/bin/env python
import rospy
import numpy as np
import math

from sensor_msgs.msg import Imu

IMU_RAW_TOPIC = '/mavros/imu/data'
IMU_DEST_TOPIC = 'sensors/imu/imu'
NODE_NAME = 'imu_pub'

imu_pub = rospy.Publisher(IMU_DEST_TOPIC, Imu, queue_size=50)

def change_frame_id(msg):
    msg.header.frame_id = "imu_link"
    imu_pub.publish(msg)

def listener():
    rospy.init_node(NODE_NAME)
    rospy.Subscriber(IMU_RAW_TOPIC, Imu, change_frame_id)
    rospy.spin()

if __name__ == '__main__':
    listener()
