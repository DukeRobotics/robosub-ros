import os
from glob import glob
from setuptools import setup

package_name = 'simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('./launch/*.launch.py')),
        (os.path.join('share', package_name, 'data'), glob('./data/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Duke Robotics Club',
    maintainer_email='hello@duke-robotics.com',
    description='Physics-enabled simulation that can be used for local testing.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'square_command = simulation.square_command:main',
            'sim = simulation.sim_loop:main',
            'fake_cv_maker = simulation.fake_cv_maker:main',
        ],
    },
)
