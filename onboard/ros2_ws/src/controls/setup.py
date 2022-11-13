import os
from glob import glob
from setuptools import setup

package_name = 'controls'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('./launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('./config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Duke Robotics Club',
    maintainer_email='hello@duke-robotics.com',
    description='A package that handles the control algorithms for the robot.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'desired_state = controls.pid.desired_state:main',
            'pid = controls.pid.pid:main',
            'state_republisher = controls.state_republisher:main',
            'test_state_publisher = controls.test_state_publisher:main',
            'thruster_controls = controls.thruster.thruster_controls:main',
        ],
    },
)
