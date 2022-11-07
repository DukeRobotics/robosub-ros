from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    dvl_raw = Node(package='data_pub',
                   executable='dvl_raw',
                   name='dvl_raw',
                   output='screen',
                   emulate_tty=True)

    dvl_odom = Node(package='data_pub',
                    executable='dvl_to_odom',
                    name='dvl_odom',
                    output='screen',
                    emulate_tty=True)
    ld.add_action(dvl_raw)
    ld.add_action(dvl_odom)

    return ld
