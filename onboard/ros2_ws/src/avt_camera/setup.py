import os
from glob import glob
from setuptools import setup

package_name = 'avt_camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('./launch/*.launch.py')),
        (os.path.join('share', package_name, 'calibrations'), glob('./calibrations/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Duke Robotics Club',
    maintainer_email='hello@duke-robotics.com',
    description='A package that contains driver code for Allied Vision GigE cameras.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mono_camera = avt_camera.mono_camera:main',
            'stereo_camera = avt_camera.stereo_camera:main',
        ],
    },
)
