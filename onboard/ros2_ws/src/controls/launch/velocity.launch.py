import yaml
import resource_retriever as rr
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.conditions import IfCondition, UnlessCondition

SIM_ARG_NAME = 'sim'
AXES = ['x', 'y', 'z', 'roll', 'pitch', 'yaw']
AXIS_PLACEHOLDER = '{{ axis }}'


def sub_axis(axis, arg):
    return arg.replace(AXIS_PLACEHOLDER, axis)


def create_pid_node(ld, axis, args):
    name = sub_axis(axis, args['name'])
    enable_service = sub_axis(axis, args['enable'])
    reset_service = sub_axis(axis, args['reset'])
    effort = sub_axis(axis, args['effort'])
    state = sub_axis(axis, args['state'])
    setpoint = sub_axis(axis, args['setpoint'])
    sim_node = Node(package='controls',
                    executable='pid',
                    namespace='controls/pid',
                    name=name,
                    output='screen',
                    emulate_tty=True,
                    parameters=[
                        {'Kp': args['pid'][axis]['sim']['kp']},
                        {'Ki': args['pid'][axis]['sim']['ki']},
                        {'Kd': args['pid'][axis]['sim']['kd']},
                        {'upper_limit': args['pid'][axis]['upper_limit']},
                        {'lower_limit': args['pid'][axis]['lower_limit']},
                        {'enable_service': enable_service},
                        {'reset_service': reset_service},
                        {'effort': effort},
                        {'state': state},
                        {'setpoint': setpoint},
                    ],
                    condition=IfCondition(LaunchConfiguration(SIM_ARG_NAME)))
    default_node = Node(package='controls',
                        executable='pid',
                        namespace='controls/pid',
                        name=name,
                        output='screen',
                        emulate_tty=True,
                        parameters=[
                            {'Kp': args['pid'][axis]['robot']['kp']},
                            {'Ki': args['pid'][axis]['robot']['ki']},
                            {'Kd': args['pid'][axis]['robot']['kd']},
                            {'upper_limit': args['pid'][axis]['upper_limit']},
                            {'lower_limit': args['pid'][axis]['lower_limit']},
                            {'enable_service': enable_service},
                            {'reset_service': reset_service},
                            {'effort': effort},
                            {'state': state},
                            {'setpoint': setpoint},
                        ],
                        condition=UnlessCondition(LaunchConfiguration(SIM_ARG_NAME)))
    ld.add_action(sim_node)
    ld.add_action(default_node)


def generate_launch_description():
    ld = LaunchDescription()
    sim = LaunchConfiguration(SIM_ARG_NAME)
    sim_arg = DeclareLaunchArgument(
        SIM_ARG_NAME,
        description='Set to true if running in simulation',
        default_value=TextSubstitution(text="false"))
    ld.add_action(sim_arg)

    file = rr.get_filename('package://controls/config/velocity.yaml', use_protocol=False)
    with open(file) as f:
        config = yaml.safe_load(f)

    for axis in AXES:
        create_pid_node(ld, axis, config)

    return ld
