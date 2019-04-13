#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import String
from data_pub.msg import DVLRaw
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3


odom_pub = rospy.Publisher('odom', Odometry, queue_size=50)

def callback(msg):
    # handle message here
    odom = Odometry()
    current_time = rospy.Time.now()
    odom.header.stamp = current_time
    odom.header.frame_id = 'odom'

    # be position
    x = np.float64(msg.be_east)
    y = np.float64(msg.be_north)
    z = np.float64(msg.be_upwards)

    # bs velocity
    vx = np.float64(msg.bs_transverse)
    vy = np.float64(msg.bs_longitudinal) 
    vz = np.float64(msg.bs_normal)

    # quat
    roll = np.float64(msg.sa_roll)
    pitch = np.float64(msg.sa_pitch)
    yaw = np.float64(msg.sa_heading)
    odom_quat = quaternion_from_euler(yaw,pitch,roll)



    # set pose
    odom.pose.pose = Pose(Point(x, y, z), Quaternion(*odom_quat))
    odom.child_frame_id = "base_link"
    # set twist (angular velocity to 0,0,0)
    odom.twist.twist = Twist(Vector3(vx, vy, vz), Vector3(0, 0, 0))
    odom_pub.publish(odom)

    
    
def listener():   
    #initialize subscirber
    rospy.init_node("dvl_listener")
    rospy.Subscriber("dvl_raw", DVLRaw, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()
    