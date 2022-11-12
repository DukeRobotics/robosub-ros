import os
from glob import glob
from setuptools import setup

package_name = 'data_pub'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('./launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Duke Robotics Club',
    maintainer_email='hello@duke-robotics.com',
    description='Package that reads and parses through sensor data.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'dvl_raw = data_pub.dvl_raw:main',
            'dvl_to_odom = data_pub.dvl_to_odom:main',
            'imu = data_pub.imu:main',
            'pressure_converter = data_pub.pressure_converter:main',
        ],
    },
)
