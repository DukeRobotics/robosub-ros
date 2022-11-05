import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    base_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('simulation'), 'launch'),
            '/base_sim.launch.py'])
    )
    square_node = Node(
        package='simulation',
        executable='square_command',
        name='square_command',
        emulate_tty=True)

    ld.add_action(base_sim)
    ld.add_action(square_node)
    return ld
