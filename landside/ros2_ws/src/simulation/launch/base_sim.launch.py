from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    sim_node = Node(
        package='simulation',
        executable='sim',
        name='simulation',
        output='screen',
        emulate_tty=True)
    fake_cv_node = Node(
        package='simulation',
        executable='fake_cv_maker',
        name='sim_fake_cv_maker',
        output='screen',
        emulate_tty=True)

    ld.add_action(sim_node)
    ld.add_action(fake_cv_node)

    return ld 
