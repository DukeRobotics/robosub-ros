from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    imu = Node(package='data_pub',
               executable='imu',
               name='imu_pub',
               output='screen',
               emulate_tty=True)
    ld.add_action(imu)
    return ld
