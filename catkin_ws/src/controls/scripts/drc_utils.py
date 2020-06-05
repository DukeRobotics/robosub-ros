from tf.transformations import euler_from_quaternion, quaternion_multiply, quaternion_conjugate


def get_directions():
    return ['x', 'y', 'z', 'roll', 'pitch', 'yaw']


def get_pose_topic(direction):
    return '/controls/state/pose/' + direction


def get_twist_topic(direction):
    return '/controls/state/twist/' + direction


def get_pid_topic(direction):
    return 'controls/' + direction + '_pos/setpoint'


def get_pid_enable(direction):
    return 'controls/enable/' + direction + '_pos'


def get_power_topic(direction):
    return '/controls/power/' + direction


def get_controls_move_topic(direction):
    return '/control_effort/' + direction


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


def publish_data_dictionary(publishers, indexes, vals):
    for d in indexes:
        publishers[d].publish(vals[d])


def publish_data_constant(publishers, indexes, val):
    for d in indexes:
        publishers[d].publish(val)
