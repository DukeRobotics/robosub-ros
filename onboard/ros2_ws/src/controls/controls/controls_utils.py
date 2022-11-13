from std_msgs.msg import Float64
from tf_transformations import euler_from_quaternion, quaternion_multiply, quaternion_conjugate
from geometry_msgs.msg import Vector3Stamped, Twist, PoseStamped
from tf2_geometry_msgs import do_transform_pose, do_transform_vector3


def get_axes():
    return ['x', 'y', 'z', 'roll', 'pitch', 'yaw']


def get_effort_topic(axis):
    return '/controls/effort/' + axis


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


def transform_pose(pose, transform):
    """Transforms a ROS pose into another reference frame.

    Args:
        pose: The ROS pose that will be transformed
        transform: The tf2 transform object

    Returns:
        A new ROS pose transformed according to the transform
    """

    return do_transform_pose(pose, transform)


def transform_twist(twist, transform):
    """Transforms a ROS twist into another reference frame.

    Args:
        twist: The ROS twist that will be transformed
        transform: The tf2 transform object

    Returns:
        A new ROS twist transformed according to the transform
    """
    lin = Vector3Stamped()
    ang = Vector3Stamped()

    lin.vector = twist.linear
    ang.vector = twist.angular

    twist_tf = Twist()
    twist_tf.linear = do_transform_vector3(lin, transform).vector
    twist_tf.angular = do_transform_vector3(ang, transform).vector

    return twist_tf


def publish_data_dictionary(publishers, vals, indexes=get_axes()):
    """ Publish a dictionary of floats """
    for d in indexes:
        publishers[d].publish(Float64(data=vals[d]))


def publish_data_constant(publishers, val, indexes=get_axes()):
    """ Publish a float msg to all publishers """
    for d in indexes:
        publishers[d].publish(Float64(data=val))
