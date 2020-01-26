import numpy as np
from geometry_msgs.msg import Vector3, Quaternion, Odometry, Pose
from tf.transformations import euler_from_quaterion
import math
import tf2_ros
from task_runner import TaskRunner

def linear_distance(point1, point2):
    """Find linear distance between two points.

    Arguments:
    point1: geometry_msgs/Point representing location 1
    point2: geometry_msgs/Point representing location 2

    Returns:
    magnitude of the linear distance between the points
    """
    vector1 = np.array([point1.x, point1.y, point1.z])
    vector2 = np.array([point2.x, point2.y, point2.z])
    distance = np.linalg.norm(vector2 - vector1)
    return distance

def angular_distance_quat(quat1, quat2):
    """Find the difference between two orientations (quaternions).

    Arguments:
    quat1: geometry_msgs/Quaternion of orientation 1
    quat2: geometry_msgs/Quaternion of orientation 2

    Returns:
    geometry_msgs/Vector3 of the magnitude of the two orientations' differences in each axis (roll, pitch, yaw), in radians
    """
    #convert quat1 and quat2 to lists
    quat1 = [quat1.x, quat1.y, quat1.z, quat1.w]
    quat2 = [quat2.x, quat2.y, quat2.z, quat2.w]
    rpy1 = euler_from_quaterion(quat1)
    rpy2 = euler_from_quaternion(quat2)
    return angular_distance_rpy(rpy1, rpy2)

def angular_distance_rpy(rpy1, rpy2):
    """Find the difference between two orientations (roll-pitch-yaw).

    Arguments:
    rpy1: list of orientation 1 (roll, pitch, yaw), in radians
    rpy2: list of orientation 2 (roll, pitch, yaw), in radians

    Returns:
    geometry_msgs/Vector3 of the magnitude of the two orientations' differences in each axis
    """
    roll = math.fabs(rpy1[0] - ryp2[0])
    pitch = math.fabs(rpy1[1] - rpy2[1])
    yaw = math.fabs(rpy1[2] - rpy2[2])
    return Vector3(roll, pitch, yaw)

def at_pose(current_pose, desired_pose, linear_tol=0.1, angular_tol=3):
    """Check if within tolerance of a pose (position and orientation).

    Arguments:
    current_pose: geometry_msgs/Pose representing current pose
    desired_pose: geometry_msgs/Pose representing pose to check with

    Keyword Arugments:
    linear_tol: allowable linear distance for robot to be considered at pose
    angular_tol: allowable angular tolerance (radians) for robot to be considered at pose, applied to all axes

    Returns:
    Boolean that is true if current_pose is within tolerances of desired_pose
    """
    linear = linear_distance(current_pose.point, desired_pose.point) < linear_tol
    angular_dist = angular_distance_quat(current_pose.orientation, desired_pose.orientation)
    angular = np.all(np.array([angular_dist.x, angular_dist.y, angular_dist.z]) < (np.ones((3)) * angular_tol))
    return (linear and angular)

def transform(origin, destination, odometry=None, pose=None):
    """Transforms Odometry input from origin frame to destination frame
    
    Arguments:
    origin: the starting frame
    destination: the frame to trasform to
    odometry: the odometry message to transform

    Returns:
    The transformed odometry message
    """
    if(odometry != None):
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        trans = tfBuffer.lookup_transform(origin, destination, rospy.Time(0))
        #TODO: transform odometry    
    elif(pose != None):
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        trans = tfBuffer.lookup_transform(origin, destination, rospy.Time(0))
        #TODO: transform pose

    return transformed 

def publish_desired_pose_global(pose):
    TaskRunner.DESIRED_POSE_GLOBAL_PUBLISHER.publish(pose)

def publish_desired_twist_global(twist):
    TaskRunner.DESIRED_TWIST_GLOBAL_PUBLISHER.publish(twist)

def publish_desired_twist_local(twist):
    TaskRunner.DESIRED_TWIST_LOCAL_PUBLISHER.publish(twist)
