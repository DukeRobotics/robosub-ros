from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, TextSubstitution

SIM_ARG_NAME = 'sim'


def wrapper(sim):
    return Node(
        package='acoustics',
        executable='wrapper',
        name='acoustics_wrapper',
        parameters=[
            {"sim": sim}
        ],
        output='screen',
        emulate_tty=True)


def data_generator():
    return Node(
        package='acoustics',
        executable='data_server',
        name='acoustics_data_generator',
        output='screen',
        emulate_tty=True)


def saleae():
    return Node(
        package='acoustics',
        executable='saleae_interface',
        name='saleae',
        output='screen',
        emulate_tty=True)


def guesser():
    return Node(
        package='acoustics',
        executable='guess_server',
        name='acoustics_guesser',
        output='screen',
        emulate_tty=True)


def processor():
    return Node(
        package='acoustics',
        executable='processing_server',
        name='acoustics_processor',
        output='screen',
        emulate_tty=True)


def generate_launch_description():
    ld = LaunchDescription()
    # Add joystick_type argument to launch config
    sim = LaunchConfiguration(SIM_ARG_NAME)
    sim_arg = DeclareLaunchArgument(
        SIM_ARG_NAME,
        description='Set to true if running in simulation',
        default_value=TextSubstitution(text="false"))
    ld.add_action(sim_arg)
    ld.add_action(wrapper(sim))

    if sim:
        ld.add_action(data_generator())
    else:
        ld.add_action(saleae())

    ld.add_action(guesser())
    ld.add_action(processor())

    return ld
