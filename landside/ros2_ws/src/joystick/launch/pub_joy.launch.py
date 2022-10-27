from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

JOYSTICK_TYPE_ARG_NAME = 'joystick_type'


def generate_launch_description():
    ld = LaunchDescription()
    # Add joystick_type argument to launch config
    joystick_type = LaunchConfiguration(JOYSTICK_TYPE_ARG_NAME)
    joystick_type_arg = DeclareLaunchArgument(
        JOYSTICK_TYPE_ARG_NAME,
        description='Type of joystick in use')
    joy_pub = Node(
        package='joystick',
        executable='joy_pub',
        name='joy_pub',
        parameters=[
            {"joy_pub/joystick_type": joystick_type}
        ])
    ld.add_action(joystick_type_arg)
    ld.add_action(joy_pub)
    return ld 
