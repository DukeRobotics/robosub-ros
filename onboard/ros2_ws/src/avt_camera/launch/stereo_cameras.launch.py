from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    camera = Node(package='avt_camera',
                  executable='stereo_camera',
                  name='stereo',
                  parameters=[
                      {'camera': ['left', 'right']},
                      {'camera_id': ['DEV_000F315C1ED5', 'DEV_000F315C1ED8']}
                  ],
                  output='screen',
                  emulate_tty=True)
    ld.add_action(camera)
    return ld
