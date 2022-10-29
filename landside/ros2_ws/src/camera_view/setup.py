import os
from glob import glob
from setuptools import setup

package_name = 'camera_view'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'avi'), glob('./avi/*.avi')),
        (os.path.join('share', package_name, 'bag'), glob('./bag/*.bag')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Duke Robotics Club',
    maintainer_email='hello@duke-robotics.com',
    description='Package that allows for viewing, saving, and loading videos to simulate camera input.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'video_to_bag = camera_view.video_to_bag:main',
            'bag_to_video = camera_view.bag_to_video:main',
        ],
    },
)
