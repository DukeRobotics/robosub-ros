#!/usr/bin/env python

import rospy
import numpy as np
import math

from std_msgs.msg import String
from data_pub.msg import DVLRaw
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

NODE_NAME = 'dvl_odom_pub'
DVL_RAW_TOPIC = 'sensors/dvl/raw'
DVL_ODOM_TOPIC = 'sensors/dvl/odom'

DVL_BAD_STATUS_MSG = 'V'

odom_pub = rospy.Publisher(DVL_ODOM_TOPIC, Odometry, queue_size=50)


def callback(msg):
    # check if the data is good
    # for now, only check bs and sa status as they are the only two data that we are currently using
    # there is no status for sa
    # for status: A = good, V = bad
    if msg.bs_status == DVL_BAD_STATUS_MSG:
        return
    # handle message here
    odom = Odometry()
    current_time = rospy.Time.now()
    odom.header.stamp = current_time
    odom.header.frame_id = 'odom'

    # Position data does not exist, is set to 0 here and should not be used
    x = 0
    y = 0
    z = 0

    # bs velocity, normalized to meters (given in mm)
    vx = -np.float64(msg.bs_longitudinal) / 1000
    vy = np.float64(msg.bs_transverse) / 1000
    vz = np.float64(msg.bs_normal) / 1000

    # quat
    roll = math.radians(np.float64(msg.sa_roll))
    pitch = math.radians(np.float64(msg.sa_pitch))
    yaw = math.radians(np.float64(msg.sa_heading))
    odom_quat = quaternion_from_euler(roll, pitch, yaw)

    # set pose
    odom.pose.pose = Pose(Point(x, y, z), Quaternion(*odom_quat))
    odom.child_frame_id = "dvl_link"
    # set twist (set angular velocity to (0, 0, 0), should not be used)
    odom.twist.twist = Twist(Vector3(vx, vy, vz), Vector3(0, 0, 0))
    odom_pub.publish(odom)


def listener():
    rospy.init_node(NODE_NAME)
    rospy.Subscriber(DVL_RAW_TOPIC, DVLRaw, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()
