import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = LaunchDescription()
    dvl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('data_pub'), 'launch'),
            '/pub_dvl.launch.py'])
    )
    imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('data_pub'), 'launch'),
            '/pub_imu.launch.py'])
    )
    depth = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('data_pub'), 'launch'),
            '/pub_depth.launch.py'])
    )

    ld.add_action(dvl)
    ld.add_action(imu)
    ld.add_action(depth)
    return ld
