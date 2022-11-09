import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()
    ekf_config = os.path.join(get_package_share_directory(
        'sensor_fusion'), 'params', 'ekf.yaml')

    # Start robot localization using an Extended Kalman filter
    robot_localization = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        emulate_tty=True,
        parameters=[ekf_config],
        remappings=[
            ("odometry/filtered", "state")
        ])
    # TODO: Do we need to republish the robot urdf here?

    ld.add_action(robot_localization)
    return ld
