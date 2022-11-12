from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, TextSubstitution


def generate_launch_description():
    return LaunchDescription([Node(package='cv',
                                  executable='detector',
                                  name=f'detector_right',
                                  parameters=[
                                      {'camera': 'right'}
                                  ],
                                  output='screen',
                                  emulate_tty=True)])
