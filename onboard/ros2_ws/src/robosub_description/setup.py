import os
from glob import glob
from setuptools import setup

package_name = 'robosub_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'meshes'), glob('./meshes/*.STL')),
        (os.path.join('share', package_name, 'urdf'), glob('./urdf/*.urdf')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Duke Robotics Club',
    maintainer_email='hello@duke-robotics.com',
    description='A package that contains relevant data and models about our robot.',
    license='MIT',
)
