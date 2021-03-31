from tf.transformations import euler_from_quaternion, quaternion_multiply, quaternion_conjugate
from geometry_msgs.msg import Vector3Stamped, Twist, PoseStamped
from custom_msgs.msg import TopicNames

def get_axes():
    return ['x', 'y', 'z', 'roll', 'pitch', 'yaw']


def get_pose_topic(axis):
    return TopicNames.controls_state_pose.format(axis)

def get_twist_topic(axis):
    return TopicNames.controls_state_twist.format(axis)


def get_vel_topic(axis):
    return TopicNames.controls_vel_setpoint.format(axis)


def get_pid_topic(axis):
    return TopicNames.controls_pos_setpoint.format(axis)


def get_pos_pid_enable(axis):
    return TopicNames.controls_enable_pos.format(axis)


def get_vel_pid_enable(axis):
    return TopicNames.controls_enable_vel.format(axis)


def get_power_topic(axis):
    return TopicNames.controls_power.format(axis)


def get_controls_move_topic(axis):
    return TopicNames.control_effort.format(axis)


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
    pose_stamped = PoseStamped()
    pose_stamped.pose = pose
    pose_stamped.header.frame_id = base_frame

    return listener.transformPose(target_frame, pose_stamped).pose


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
