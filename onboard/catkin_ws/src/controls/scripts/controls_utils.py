from tf.transformations import euler_from_quaternion, quaternion_multiply, quaternion_conjugate
from geometry_msgs.msg import Vector3Stamped, Twist, PoseStamped


def get_axes():
    return ['x', 'y', 'z', 'roll', 'pitch', 'yaw']


def get_controls_move_topic(axis):
    return '/control_effort/' + axis


def get_power_topic(axis):
    return '/controls/power/' + axis


def parse_pose(pose):
    """Converts a ROS pose message to a dictionary that maps direction to value. Does a transformation from quaternion
    to euler to convert orientation data to euler angles used by PID loops.

    Args:
        pose: ROS pose message

    Returns:
        TYPE: Dictionary that maps direction to value for each axis in pose
    """
    pose_dict = {'x': pose.position.x, 'y': pose.position.y, 'z': pose.position.z}
    pose_dict['roll'], pose_dict['pitch'], pose_dict['yaw'] = euler_from_quaternion(
        [pose.orientation.x,
         pose.orientation.y,
         pose.orientation.z,
         pose.orientation.w])
    return pose_dict


def parse_twist(twist):
    """Converts a ROS twist message to a dictionary that maps direction to value

    Args:
        twist: ROS twist message

    Returns:
        TYPE: Dictionary that maps direction to value for each axis in twist
    """
    twist_dict = {'x': twist.linear.x,
                  'y': twist.linear.y,
                  'z': twist.linear.z,
                  'roll': twist.angular.x,
                  'pitch': twist.angular.y,
                  'yaw': twist.angular.z}
    return twist_dict


def quat_vec_mult(q1, v1):
    """Rotate vector v1 by quaternion q1, and return the resulting vector.
    From https://answers.ros.org/question/196149/how-to-rotate-vector-by-quaternion-in-python/
    """
    q2 = list(v1)
    q2.append(0.0)
    return quaternion_multiply(
        quaternion_multiply(q1, q2),
        quaternion_conjugate(q1)
    )[:3]


def transform_pose(listener, base_frame, target_frame, pose):
    """Transforms a ROS pose into another reference frame.

    Args:
        listener: The ROS TransformListener that retrieves transformation data
        base_frame: The initial reference frame
        target_frame: The target reference frame
        pose: The ROS pose that will be transformed

    Returns:
        A new ROS pose transformed into the target frame
    """
    pose_stamped = PoseStamped()
    pose_stamped.pose = pose
    pose_stamped.header.frame_id = base_frame

    return listener.transformPose(target_frame, pose_stamped).pose


def transform_twist(listener, base_frame, target_frame, twist):
    """Transforms a ROS twist into another reference frame.

    Args:
        listener: The ROS TransformListener that retrieves transformation data
        base_frame: The initial reference frame
        target_frame: The target reference frame
        twist: The ROS twist that will be transformed

    Returns:
        A new ROS twist transformed into the target frame
    """
    lin = Vector3Stamped()
    ang = Vector3Stamped()

    lin.vector = twist.linear
    ang.vector = twist.angular

    lin.header.frame_id = base_frame
    ang.header.frame_id = base_frame

    twist_tf = Twist()
    twist_tf.linear = listener.transformVector3(target_frame, lin).vector
    twist_tf.angular = listener.transformVector3(target_frame, ang).vector

    return twist_tf


def publish_data_dictionary(publishers, vals, indexes=get_axes()):
    for d in indexes:
        publishers[d].publish(vals[d])


def publish_data_constant(publishers, val, indexes=get_axes()):
    for d in indexes:
        publishers[d].publish(val)
