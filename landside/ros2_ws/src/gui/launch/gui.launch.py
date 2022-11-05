from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    gui = Node(
        package='rqt_gui',
        executable='rqt_gui',
        name='gui',
        output='screen',
        arguments=[
            {"perspective-file": "$(find gui)/share/gui/config/robosub_ros_gui.perspective"}
        ])
    ld.add_action(gui)
    return ld 
