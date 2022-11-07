import os
from glob import glob
from setuptools import setup

package_name = 'acoustics'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('./launch/*.launch.py')),
        (os.path.join('share', package_name, 'data'), glob('./data/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Duke Robotics Club',
    maintainer_email='hello@duke-robotics.com',
    description='This package performs the processing on acoustics signals.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'wrapper = acoustics.wrapper:main',
            'data_server = acoustics.data_server:main',
            'guess_server = acoustics.guess_server:main',
            'processing_server = acoustics.processing_server:main',
            'saleae = acoustics.saleae_interface:main',
        ],
    },
)
