#!/usr/bin/env python
import rospy
import numpy as np
import math

IMU_RAW_TOPIC = '/mavros/imu/data'
IMU_DEST_TOPIC = 'sensors/imu/imu'

def change_frame_id(msg):
    msg.header.frame_id = "imu_link"
    msg.publish(msg)

def listener():
    rospy.init_node(NODE_NAME)
    rospy.Subscriber()
    rospy.spin()
