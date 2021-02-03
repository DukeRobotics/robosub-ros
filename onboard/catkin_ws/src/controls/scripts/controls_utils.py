from tf.transformations import euler_from_quaternion, quaternion_multiply, quaternion_conjugate
import numpy as np
from geometry_msgs.msg import Vector3Stamped, Twist


def get_axes():
    return ['x', 'y', 'z', 'roll', 'pitch', 'yaw']


def get_pose_topic(axis):
    return '/controls/state/pose/' + axis


def get_twist_topic(axis):
    return '/controls/state/twist/' + axis


def get_vel_topic(axis):
    return 'controls/' + axis + '_vel/setpoint'


def get_pid_topic(axis):
    return 'controls/' + axis + '_pos/setpoint'


def get_pos_pid_enable(axis):
    return 'controls/enable/' + axis + '_pos'


def get_vel_pid_enable(axis):
    return 'controls/enable/' + axis + '_vel'


def get_power_topic(axis):
    return '/controls/power/' + axis


def get_controls_move_topic(axis):
    return '/control_effort/' + axis


def parse_pose(pose):
    pose_dict = {'x': pose.position.x, 'y': pose.position.y, 'z': pose.position.z}
    pose_dict['roll'], pose_dict['pitch'], pose_dict['yaw'] = euler_from_quaternion(
        [pose.orientation.x,
         pose.orientation.y,
         pose.orientation.z,
         pose.orientation.w])
    return pose_dict


def parse_twist(twist):
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
    return listener.transformPose(base_frame, target_frame, pose)


def transform_twist(listener, base_frame, target_frame, twist):
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


def publish_data_dictionary(publishers, indexes, vals):
    for d in indexes:
        publishers[d].publish(vals[d])


def publish_data_constant(publishers, indexes, val):
    for d in indexes:
        publishers[d].publish(val)
