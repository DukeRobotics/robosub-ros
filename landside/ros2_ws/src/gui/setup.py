import os
from glob import glob
from setuptools import setup

package_name = 'gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('./config/*.perspective')),
        (os.path.join('share', package_name, 'launch'), glob('./launch/*.launch.py')),
        (os.path.join('share', package_name, 'resource'), glob('./resource/*.ui')),
        (os.path.join('share', package_name, 'plugins'), glob('./plugins/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Duke Robotics Club',
    maintainer_email='hello@duke-robotics.com',
    description='Provides a GUI to interact with the robot',
    license='MIT',
    entry_points={
        'console_scripts': [],
    },
)

