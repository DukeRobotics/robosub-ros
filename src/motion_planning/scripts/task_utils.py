import numpy as np
from geometry_msgs.msg import Vector3, Quaternion, Pose, PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math
import tf2_ros
import tf2_geometry_msgs
import rospy

def linear_distance(point1, point2):
    """Find linear distance between two points.

    Parameters:
    point1 (geometry_msgs/Point): location 1
    point2 (geometry_msgs/Point): location 2

    Returns:
    float: magnitude of the linear distance between the points
    """

    vector1 = np.array([point1.x, point1.y, point1.z])
    vector2 = np.array([point2.x, point2.y, point2.z])
    distance = np.linalg.norm(vector2 - vector1)
    return distance

def angular_distance_quat(quat1, quat2):
    """Find the difference between two orientations (quaternions).

    Parameters:
    quat1 (geometry_msgs/Quaternion): orientation 1
    quat2 (geometry_msgs/Quaternion): orientation 2

    Returns:
    geometry_msgs/Vector3: magnitude of the two orientations' differences in each axis (roll, pitch, yaw), in radians
    """
    #convert quat1 and quat2 to lists
    quat1 = [quat1.x, quat1.y, quat1.z, quat1.w]
    quat2 = [quat2.x, quat2.y, quat2.z, quat2.w]
    rpy1 = euler_from_quaterion(quat1)
    rpy2 = euler_from_quaternion(quat2)
    return angular_distance_rpy(rpy1, rpy2)

def angular_distance_rpy(rpy1, rpy2):
    """Find the difference between two orientations (roll-pitch-yaw).

    Parameters:
    rpy1 (list): orientation 1 (roll, pitch, yaw), in radians
    rpy2 (list): orientation 2 (roll, pitch, yaw), in radians

    Returns:
    geometry_msgs/Vector3: magnitude of the two orientations' differences in each axis
    """
    roll = math.fabs(rpy1[0] - ryp2[0])
    pitch = math.fabs(rpy1[1] - rpy2[1])
    yaw = math.fabs(rpy1[2] - rpy2[2])
    return Vector3(roll, pitch, yaw)

def at_pose(current_pose, desired_pose, linear_tol=0.1, angular_tol=3):
    """Check if within tolerance of a pose (position and orientation).

    Paramters:
    current_pose (geometry_msgs/Pose): current pose
    desired_pose (geometry_msgs/Pose): pose to check with

    Keyword Arguments:
    linear_tol (float): allowable linear distance for robot to be considered at pose
    angular_tol (float): allowable angular tolerance (radians) for robot to be considered at pose, applied to all axes

    Returns:
    Boolean: true if current_pose is within tolerances of desired_pose
    """
    linear = linear_distance(current_pose.point, desired_pose.point) < linear_tol
    angular_dist = angular_distance_quat(current_pose.orientation, desired_pose.orientation)
    angular = np.all(np.array([angular_dist.x, angular_dist.y, angular_dist.z]) < (np.ones((3)) * angular_tol))
    return (linear and angular)

def transform(origin_frame, dest_frame, poseORodom):
    """Transforms poseORodom from origin_frame to dest_frame frame

    Arguments:
    origin_frame: the starting frame
    dest_frame: the frame to trasform to
    poseORodom: the Pose, PoseStamped, Odometry, or OdometryStamped message to transform

    Returns:
    Pose, PoseStamped, Odometry, or OdometryStamped: The transformed position
    """

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    trans = tfBuffer.lookup_transform(dest_frame, origin_frame, rospy.Time(), rospy.Duration(0.5))
    
    if isinstance(poseORodom, PoseStamped):
        transformed = tf2_geometry_msgs.do_transform_pose(poseORodom, trans)
        return transformed
    
    elif isinstance(poseORodom, Pose):
        temp_pose_stamped = PoseStamped()
        temp_pose_stamped.pose = poseORodom
        transformed = tf2_geometry_msgs.do_transform_pose(temp_pose_stamped, trans)
        return transformed.pose

    else:
        #add wrong message type error here?
        return