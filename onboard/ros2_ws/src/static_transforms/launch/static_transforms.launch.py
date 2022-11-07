from launch import LaunchDescription
from launch_ros.actions import Node


CORNER_LINK_TRANSFORM = [0.1080, 0.1014, 0.1575, 0, 0, 0, 'base_link', 'corner_link']
DVL_LINK_TRANSFORM = [-0.3118, -0.1014, -0.3894, 0, 0, 0, 'corner_link', 'dvl_link']
IMU_LINK_TRANSFORM = [-0.0359, -0.1008, -0.1219, 0, 0, 0, 'corner_link', 'imu_link']
CAMERAS_LINK_TRANSFORM = [0.2287, -0.2061, 0.0407, 0, 0, 0, 'corner_link', 'cameras_link']
LEFT_CAMERAS_LINK_TRANSFORM = [0, 0.0362, 0, 0, 0, 0, 'cameras_link', 'left_cameras_link']
RIGHT_CAMERAS_LINK_TRANSFORM = [0, -0.0362, 0, 0, 0, 0, 'cameras_link', 'right_cameras_link']


def make_transform_publisher(transform):
    return Node(package='tf2_ros',
                executable='static_transform_publisher',
                arguments=['--x', str(transform[0]),
                           '--y', str(transform[1]),
                           '--z', str(transform[2]),
                           '--roll', str(transform[3]),
                           '--pitch', str(transform[4]),
                           '--yaw', str(transform[5]),
                           '--frame-id', transform[6],
                           '--child-frame-id', transform[7]])


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(make_transform_publisher(CORNER_LINK_TRANSFORM))
    ld.add_action(make_transform_publisher(DVL_LINK_TRANSFORM))
    ld.add_action(make_transform_publisher(IMU_LINK_TRANSFORM))
    ld.add_action(make_transform_publisher(CAMERAS_LINK_TRANSFORM))
    ld.add_action(make_transform_publisher(LEFT_CAMERAS_LINK_TRANSFORM))
    ld.add_action(make_transform_publisher(RIGHT_CAMERAS_LINK_TRANSFORM))
    return ld
