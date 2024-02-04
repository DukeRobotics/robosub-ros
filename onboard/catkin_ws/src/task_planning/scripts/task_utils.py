import numpy as np
import rospy
import smach
import tf2_geometry_msgs
import tf2_ros
from geometry_msgs.msg import Vector3, Pose, PoseStamped, PoseWithCovariance, \
    Twist, TwistStamped, TwistWithCovariance, Point, Quaternion, PointStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from transforms3d import euler, quaternions


def linear_distance(point1, point2):
    """Find linear distance between two points.

    Parameters:
    point1 (Point): location 1
    point2 (Point): location 2

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
    quat1 (Quaternion): orientation 1
    quat2 (Quaternion): orientation 2

    Returns:
    Vector3: magnitude of the two orientations' differences in each axis (roll, pitch, yaw), in radians
    """
    rpy1 = quat2euler([quat1.w, quat1.x, quat1.y, quat1.z])
    rpy2 = quat2euler([quat2.w, quat2.x, quat2.y, quat2.z])
    return angular_distance_rpy(rpy1, rpy2)


def angular_distance_rpy(rpy1, rpy2):
    """Find the difference between two orientations (roll-pitch-yaw).

    Parameters:
    rpy1 (list): orientation 1 (roll, pitch, yaw), in radians
    rpy2 (list): orientation 2 (roll, pitch, yaw), in radians

    Returns:
    Vector3: magnitude of the two orientations' differences in each axis
    """
    roll = np.fabs(rpy1[0] - rpy2[0])
    pitch = np.fabs(rpy1[1] - rpy2[1])
    yaw = np.fabs(rpy1[2] - rpy2[2])
    return Vector3(roll, pitch, yaw)


def at_pose(current_pose, desired_pose, linear_tol=0.1, angular_tol=3):
    """Check if within tolerance of a pose (position and orientation).

    Parameters:
    current_pose (Pose): current pose
    desired_pose (Pose): pose to check with
    linear_tol (float): allowable linear distance for robot to be considered at pose
    angular_tol (float): allowable angular tolerance (radians) for robot to be considered at pose, applied to all axes

    Returns:
    Boolean: true if current_pose is within tolerances of desired_pose
    """
    linear = linear_distance(current_pose.position, desired_pose.position) < linear_tol
    angular_dist = angular_distance_quat(current_pose.orientation, desired_pose.orientation)
    angular = np.all(np.array([angular_dist.x, angular_dist.y, angular_dist.z]) < (np.ones((3)) * angular_tol))
    return linear and angular


def at_vel(current_twist, desired_twist, linear_tol=0.1, angular_tol=0.3):
    """Check if within tolerance of a twist (linear and angular velocity)

    Parameters:
    current_twist (Twist): current twist
    desired_twist (Twist): twist to check with
    linear_tol (float): allowable linear velocity for robot to be considered at twist
    angular_tol (float): allowable angular velocity for robot to be considered at twist, applied to all axes

    Returns:
    Boolean: true if current_twist is within tolerances of desired_twist
    """

    lin_curr_vel = np.linalg.norm([current_twist.linear.x, current_twist.linear.y, current_twist.linear.z])
    lin_des_vel = np.linalg.norm([desired_twist.linear.x, desired_twist.linear.y, desired_twist.linear.z])
    linear = np.fabs(lin_curr_vel - lin_des_vel) < linear_tol

    ang_curr_vel = np.linalg.norm([current_twist.angular.x, current_twist.angular.y, current_twist.angular.z])
    ang_des_vel = np.linalg.norm([desired_twist.angular.x, desired_twist.angular.y, desired_twist.angular.z])
    angular = np.fabs(ang_curr_vel - ang_des_vel) < angular_tol

    return linear and angular


def stopped_at_pose(current_pose, desired_pose, current_twist):
    """Check if within tolerance of a pose (position and orientation) and current_twist = 0

    Parameters:
    current_pose (Pose): current pose
    desired_pose (Pose): pose to check with
    current_twist (Twist): current twist

    Returns:
    Boolean: true if stopped (current_twist = 0) at desired_pose
    """
    at_desired_pose = at_pose(current_pose, desired_pose, 0.2, 12)
    at_desired_vel = at_vel(current_twist, Twist(), 0.6, 6)

    # print("At Pose:", at_desired_pose, " At Vel:", at_desired_vel)

    return at_desired_pose and at_desired_vel


def transform_pose(tfBuffer, base_frame, target_frame, pose):
    pose_stamped = PoseStamped()
    pose_stamped.pose = pose
    pose_stamped.header.frame_id = base_frame

    return tfBuffer.transformPose(target_frame, pose_stamped).pose


def transform(origin_frame, dest_frame, poseORodom):
    """Transforms poseORodom from origin_frame to dest_frame frame

    Arguments:
    origin_frame: the starting frame
    dest_frame: the frame to trasform to
    poseORodom: the Pose, PoseStamped, Odometry, or  message to transform

    Returns:
    Pose, PoseStamped, Odometry : The transformed position
    """

    tfBuffer = tf2_ros.Buffer()
    try:
        trans = tfBuffer.lookup_transform(dest_frame, origin_frame, rospy.Time(), rospy.Duration(0.5))
    except:
        (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException)
    
    if isinstance(poseORodom, PoseStamped):
        transformed = tf2_geometry_msgs.do_transform_pose(poseORodom, trans)
        return transformed

    elif isinstance(poseORodom, Pose):
        temp_pose_stamped = PoseStamped()
        temp_pose_stamped.pose = poseORodom
        transformed = tf2_geometry_msgs.do_transform_pose(temp_pose_stamped, trans)
        return transformed.pose

    elif isinstance(poseORodom, Odometry):
        temp_pose_stamped = PoseStamped()
        temp_twist_stamped = TwistStamped()
        temp_pose_stamped.pose = poseORodom.pose.pose
        temp_twist_stamped.twist = poseORodom.twist.twist
        transformed_pose_stamped = tf2_geometry_msgs.do_transform_pose(temp_pose_stamped, trans)

        # twist as points
        twist_poseORodom = poseORodom.twist.twist
        temp_twist_point_linear = Point()
        temp_twist_point_angular = Point()
        temp_twist_point_linear.x = twist_poseORodom.linear.x
        temp_twist_point_linear.y = twist_poseORodom.linear.y
        temp_twist_point_linear.z = twist_poseORodom.linear.z
        temp_twist_point_angular.x = twist_poseORodom.angular.x
        temp_twist_point_angular.y = twist_poseORodom.angular.y
        temp_twist_point_angular.z = twist_poseORodom.angular.z
        twist_point_linear_stamped = PointStamped(point=temp_twist_point_linear)
        twist_point_angular_stamped = PointStamped(point=temp_twist_point_angular)

        # points transformed
        transformed_twist_point_linear_stamped = tf2_geometry_msgs.do_transform_point(
            twist_point_linear_stamped, trans)
        transformed_twist_point_angular_stamped = tf2_geometry_msgs.do_transform_point(
            twist_point_angular_stamped, trans)

        # points to twist
        transformed_twist = Twist()
        transformed_twist.linear.x = transformed_twist_point_linear_stamped.point.x
        transformed_twist.linear.y = transformed_twist_point_linear_stamped.point.y
        transformed_twist.linear.z = transformed_twist_point_linear_stamped.point.z
        transformed_twist.angular.x = transformed_twist_point_angular_stamped.point.x
        transformed_twist.angular.y = transformed_twist_point_angular_stamped.point.y
        transformed_twist.angular.z = transformed_twist_point_angular_stamped.point.z

        transformed_odometry = Odometry(header=Header(frame_id=dest_frame), child_frame_id=dest_frame,
                                        pose=PoseWithCovariance(pose=transformed_pose_stamped.pose),
                                        twist=TwistWithCovariance(twist=transformed_twist))
        return transformed_odometry

    else:
        rospy.logerr("Invalid message type passed to transform()")
        return None


def add_poses(pose_list):
    """Adds a list of poses

    Arguments:
    pose_list: list of poses to add

    Returns:
    Pose: the sum of the Poses
    """

    p_sum = Point(0, 0, 0)
    q_sum = [1, 0, 0, 0]

    for pose in pose_list:
        p_sum.x += pose.position.x
        p_sum.y += pose.position.y
        p_sum.z += pose.position.z

        q_sum = qmult(
            [pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z], q_sum)

    return Pose(p_sum, Quaternion(*q_sum))


def parse_pose(pose):
    pose_dict = {'x': pose.position.x, 'y': pose.position.y, 'z': pose.position.z}
    pose_dict['roll'], pose_dict['pitch'], pose_dict['yaw'] = quat2euler(
        [pose.orientation.w,
         pose.orientation.x,
         pose.orientation.y,
         pose.orientation.z])
    return pose_dict


def object_vector(cv_obj_data):
    print(cv_obj_data)
    if not cv_obj_data or cv_obj_data.label == 'none':
        return None

    return [cv_obj_data.x, cv_obj_data.y, cv_obj_data.z]


def cv_object_position(cv_obj_data):
    if not cv_obj_data or cv_obj_data.label == 'none':
        return None
    return [cv_obj_data.x, cv_obj_data.y, cv_obj_data.z]

def create_pose(x, y, z, roll, pitch, yaw):
    pose = Pose()
    pose.position = Point(x=x, y=y, z=z)
    pose.orientation = Quaternion(*euler2quat(roll, pitch, yaw))
    return pose

def local_pose_to_global(tfBuffer, pose):
    return transform_pose(tfBuffer, 'base_link', 'odom', pose)

def global_pose_to_local(tfBuffer, pose):
    return transform_pose(tfBuffer, 'odom', 'base_link', pose)
