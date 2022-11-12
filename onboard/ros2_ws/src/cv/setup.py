import os
from glob import glob
from setuptools import setup

package_name = 'cv'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('./launch/*.launch.py')),
        (os.path.join('share', package_name, 'assets'), glob('./assets/*.jpg')),
        (os.path.join('share', package_name, 'models'), glob('./models/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Duke Robotics Club',
    maintainer_email='hello@duke-robotics.com',
    description='CV package that detects objects from the camera feed.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'detector = cv.detector:main',
            'test_images = cv.test_images:main',
        ],
    },
)
