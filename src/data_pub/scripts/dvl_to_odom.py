#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import String
from data_pub.msg import DVLRaw
from nav_msgs.msg import Odometry
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

    quat = [0.0,0.0,0.0,0.0]
    # TODO: quanternion in pose, do not know how to do, uses tf
    odom.pose.pose = Pose(Point(x, y, z), quat)
    odom.child_frame_id = "base_link"
    # TODO: i have put 0 for all angular velocity, may need update
    odom.twist.twist = Twist(Vector3(vx, vy, vz), Vector3(0, 0, 0))
    odom_pub.publish(odom)

    
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    
    #initialize subscirber
    rospy.init_node("dvl_listener")
    rospy.Subscriber("dvl_raw_publisher", DVLRaw, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()
    