#!/usr/bin/env python

from geometry_msgs.msg import PoseStamped, Pose
import rospy
import tf2_ros
import tf2_geometry_msgs
import task_utils

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
    listener = tf2_ros.TransformListener(tfBuffer)
    trans = tfBuffer.lookup_transform(destination, origin, rospy.Time(), rospy.Duration(0.5))
    
    if isinstance(poseORodom, PoseStamped):
        transformed = tf2_geometry_msgs.do_transform_pose(poseORodom, trans)
        return transformed
    
    elif isinstance(poseORodom, Pose):
        temp_pose_stamped = PoseStamped()
        temp_pose_stamped.pose = poseORodom
        transformed = tf2_geometry_msgs.do_transform_pose(temp_pose_stamped, trans)
        return transformed.pose


pose_stamped = PoseStamped()
pose_stamped.pose.position.x = 2
pose_stamped.pose.position.y = 4
pose_stamped.pose.position.z = 7

transformed_pose = transform('origin1', 'destination1', pose_stamped)

print(transformed_pose.pose.position.x)
print(transformed_pose.pose.position.y)
print(transformed_pose.pose.position.z)


pose = Pose()
pose.position.x = 2
pose.position.y = 4
pose.position.z = 7

pose = transform('origin1', 'destination1', pose)
print(pose.position.x)
print(pose.position.y)
print(pose.position.z)