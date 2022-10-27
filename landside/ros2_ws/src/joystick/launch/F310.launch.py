import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = LaunchDescription()
    joystick_raw = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('joystick'), 'launch'),
            '/joystick_raw.launch.py'])
    )
    pub_joy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('joystick'), 'launch'),
            '/pub_joy.launch.py']),
        launch_arguments={'joystick_type': 'F310'}.items()
    )
    ld.add_action(joystick_raw)
    ld.add_action(pub_joy)
    return ld
