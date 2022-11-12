import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = LaunchDescription()
    left = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('cv'), 'launch'), '/left.launch.py'])
    )
    right = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('cv'), 'launch'), '/right.launch.py'])
    )
    down = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('cv'), 'launch'), '/down.launch.py'])
    )
    ld.add_action(left)
    ld.add_action(right)
    ld.add_action(down)
    return ld
