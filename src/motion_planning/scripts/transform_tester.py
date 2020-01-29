#!/usr/bin/env python

from geometry_msgs.msg import PoseStamped
import rospy
import tf2_ros
import tf2_geometry_msgs

rospy.init_node('testing')

def transform(origin, destination, this_pose=None):
    """Transforms Odometry input from origin frame to destination frame

    Arguments:
    origin: the starting frame
    destination: the frame to trasform to
    odometry: the odometry message to transform

    Returns:
    The transformed odometry message
    """

    if(this_pose != None):
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        trans = tfBuffer.lookup_transform(destination, origin, rospy.Time(), rospy.Duration(0.5))
        transformed_pose = tf2_geometry_msgs.do_transform_pose(this_pose, trans)

    return transformed_pose


pose_stamped = PoseStamped()
pose_stamped.pose.position.x = 2
pose_stamped.pose.position.y = 4
pose_stamped.pose.position.z = 7

transformed_pose = transform('origin1', 'destination1', pose_stamped)

print(transformed_pose.pose.position.x)
print(transformed_pose.pose.position.y)
print(transformed_pose.pose.position.z)
