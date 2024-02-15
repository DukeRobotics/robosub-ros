#!/usr/bin/env python3

import math
import numpy as np
import rospy
import yaml
import os
import resource_retriever as rr
from custom_msgs.msg import DVLRaw
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
from transforms3d.euler import euler2quat

CONFIG_FILE_PATH = f'package://data_pub/config/{os.getenv("ROBOT_NAME", "oogway")}.yaml'

NODE_NAME = 'dvl_odom_pub'
DVL_RAW_TOPIC = 'sensors/dvl/raw'
DVL_ODOM_TOPIC = 'sensors/dvl/odom'

DVL_BAD_STATUS_MSG = 'V'

odom_pub = rospy.Publisher(DVL_ODOM_TOPIC, Odometry, queue_size=50)
dvl_config_data = None


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
    # parentheses denote new negative signs
    vx = np.float64(msg.bs_transverse) / 1000
    vy = np.float64(msg.bs_longitudinal) / 1000
    vz = np.float64(msg.bs_normal) / 1000

    if dvl_config_data["negate_x_vel"]:
        vx = -vx
    if dvl_config_data["negate_y_vel"]:
        vy = -vy
    if dvl_config_data["negate_z_vel"]:
        vz = -vz

    # quat
    roll = math.radians(np.float64(msg.sa_roll))
    pitch = math.radians(np.float64(msg.sa_pitch))
    yaw = math.radians(np.float64(msg.sa_heading))
    odom_quat = euler2quat(roll, pitch, yaw)

    # set pose
    odom.pose.pose = Pose(Point(x, y, z), Quaternion(odom_quat[1], odom_quat[2], odom_quat[3], odom_quat[0]))
    odom.child_frame_id = "dvl_link"
    # set twist (set angular velocity to (0, 0, 0), should not be used)
    odom.twist.twist = Twist(Vector3(vx, vy, vz), Vector3(0, 0, 0))
    odom.twist.covariance[0] = 0.01
    odom.twist.covariance[7] = 0.01
    odom.twist.covariance[14] = 0.01
    odom_pub.publish(odom)


def listener():
    global dvl_config_data
    with open(rr.get_filename(CONFIG_FILE_PATH, use_protocol=False)) as f:
        config_data = yaml.safe_load(f)
        dvl_config_data = config_data['dvl']

    rospy.init_node(NODE_NAME)
    rospy.Subscriber(DVL_RAW_TOPIC, DVLRaw, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()
