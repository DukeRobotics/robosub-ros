import os
import yaml
import resource_retriever as rr
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


SIM_ARG_NAME = 'sim'
STATIC_TRANSFORM_ARG_NAME = 'transform'


def pos_pid_launch():
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('controls'), 'launch'),
            '/position.launch.py']),
        launch_arguments={'sim': LaunchConfiguration(SIM_ARG_NAME)}.items()
    )


def vel_pid_launch():
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('controls'), 'launch'),
            '/velocity.launch.py']),
        launch_arguments={'sim': LaunchConfiguration(SIM_ARG_NAME)}.items()
    )


def state_republisher():
    """ Start controls node that republishes state to separate topics """
    return Node(package='controls',
                executable='state_republisher',
                name='state_republisher',
                output='screen',
                emulate_tty=True)


def thruster_controls():
    """ Start controls node that listens to PID and outputs thruster allocations """
    return Node(package='controls',
                executable='thruster_controls',
                name='thruster_controls',
                output='screen',
                emulate_tty=True)


def desired_state():
    """ Start desired_state node that processes inputs from task planning 
    and outputs setpoints to PID """
    return Node(package='controls',
                executable='desired_state',
                name='desired_state',
                output='screen',
                emulate_tty=True)


def static_transform():
    """ Create a local/global transform manually. 
    Used for testing controls without using simulation. """
    return Node(package='tf2_ros',
                executable='static_transform_publisher',
                arguments=['--x', str(0),
                           '--y', str(0),
                           '--z', str(0),
                           '--roll', str(0),
                           '--pitch', str(0),
                           '--yaw', str(0),
                           '--frame-id', 'odom',
                           '--child-frame-id', 'base_link'],
                condition=IfCondition(LaunchConfiguration(STATIC_TRANSFORM_ARG_NAME)))


def generate_launch_description():
    ld = LaunchDescription()
    # Add simulation argument
    sim_arg = DeclareLaunchArgument(
        SIM_ARG_NAME,
        description='Set to true if running in simulation',
        default_value=TextSubstitution(text="false"))
    ld.add_action(sim_arg)
    # Add transform argument
    transform_arg = DeclareLaunchArgument(
        STATIC_TRANSFORM_ARG_NAME,
        description='Set to true if testing locally without simulation',
        default_value=TextSubstitution(text="false"))
    ld.add_action(transform_arg)
    ld.add_action(pos_pid_launch())
    ld.add_action(vel_pid_launch())
    ld.add_action(static_transform())
    ld.add_action(state_republisher())
    ld.add_action(thruster_controls())
    ld.add_action(desired_state())

    return ld
