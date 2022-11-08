import os
from glob import glob
from setuptools import setup

package_name = 'system_utils'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Duke Robotics Club',
    maintainer_email='hello@duke-robotics.com',
    description='Package that provides information about the system itself and provide utilities to help during runtime.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'remote_launch = system_utils.remote_launch:main',
            'system_info = system_utils.system_info:main',
        ],
    },
)
