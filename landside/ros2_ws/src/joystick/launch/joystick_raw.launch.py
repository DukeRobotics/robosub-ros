from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[
            {"autorepeat_rate": 20.0}
        ],
        remappings=[
            ("joy", "joystick/raw")
        ])
    ld.add_action(joy_node)
    return ld 
