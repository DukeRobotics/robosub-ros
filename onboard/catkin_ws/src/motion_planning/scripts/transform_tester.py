#!/usr/bin/env python

import rospy
import tf2_geometry_msgs
import tf2_ros
from geometry_msgs.msg import PoseStamped, Pose, Vector3Stamped
from nav_msgs.msg import Odometry

rospy.init_node('testing')


def transform(origin, destination, poseORodom):
    """Transforms Odometry input from origin frame to destination frame

    Arguments:
    origin: the starting frame
    destination: the frame to trasform to
    odometry: the odometry message to transform

    Returns:
    The transformed odometry message
    """

    tfBuffer = tf2_ros.Buffer()
    # listener = tf2_ros.TransformListener(tfBuffer)
    trans = tfBuffer.lookup_transform(destination, origin, rospy.Time(), rospy.Duration(0.5))

    if isinstance(poseORodom, PoseStamped):
        transformed = tf2_geometry_msgs.do_transform_pose(poseORodom, trans)
        return transformed

    elif isinstance(poseORodom, Pose):
        temp_pose_stamped = PoseStamped()
        temp_pose_stamped.pose = poseORodom
        transformed = tf2_geometry_msgs.do_transform_pose(temp_pose_stamped, trans)
        return transformed.pose

    elif isinstance(poseORodom, Odometry):
        temp_odom = Odometry()
        vec3 = Vector3Stamped()
        ang3 = Vector3Stamped()
        vec3.vector = poseORodom.twist.twist.linear
        ang3.vector = poseORodom.twist.twist.angular
        temp_odom.pose = tf2_geometry_msgs.do_transform_pose(poseORodom.pose, trans)
        vec3_trans = tf2_geometry_msgs.do_transform_vector3(vec3, trans)
        ang3_trans = tf2_geometry_msgs.do_transform_vector3(ang3, trans)
        temp_odom.twist.twist.linear = vec3_trans.vector
        temp_odom.twist.twist.angular = ang3_trans.vector
        return temp_odom


pose_stamped = PoseStamped()
pose_stamped.pose.position.x = 2
pose_stamped.pose.position.y = 4
pose_stamped.pose.position.z = 7

transformed_pose = transform('origin1', 'destination1', pose_stamped)
print('PoseStamped:')
print('x:', transformed_pose.pose.position.x)
print('y:', transformed_pose.pose.position.y)
print('z:', transformed_pose.pose.position.z)
print('---')

pose = Pose()
pose.position.x = 2
pose.position.y = 4
pose.position.z = 7

poser = transform('origin1', 'destination1', pose)
print('Pose:')
print('x:', poser.position.x)
print('y:', poser.position.y)
print('z:', poser.position.z)
print('---')

odom = Odometry()
odom.pose.pose.position.x = 2
odom.pose.pose.position.y = 4
odom.pose.pose.position.z = 7
odom.twist.twist.linear.x = 3
odom.twist.twist.linear.y = 5
odom.twist.twist.linear.z = 9
odom.twist.twist.angular.x = 1
odom.twist.twist.angular.y = 6
odom.twist.twist.angular.z = 8

ododom = transform('origin1', 'destination1', odom)
print('Odometry:')
print('x:', ododom.pose.pose.position.x)
print('y:', ododom.pose.pose.position.y)
print('z:', ododom.pose.pose.position.z)
print('vx:', ododom.twist.twist.linear.x)
print('vy:', ododom.twist.twist.linear.y)
print('vz:', ododom.twist.twist.linear.z)
print('wx:', ododom.twist.twist.angular.x)
print('wy:', ododom.twist.twist.angular.y)
print('wz:', ododom.twist.twist.angular.z)
print('---')
