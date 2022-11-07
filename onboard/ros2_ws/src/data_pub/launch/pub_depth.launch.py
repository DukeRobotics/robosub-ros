from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    depth_node = Node(package='data_pub',
                      executable='pressure_converter',
                      name='depth_pub',
                      output='screen',
                      emulate_tty=True)
    ld.add_action(depth_node)
    return ld
