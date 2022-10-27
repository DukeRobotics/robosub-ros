from setuptools import setup

package_name = 'pub_test'

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
    description='ROS2 publisher proof-of-concept',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = pub_test.test_publisher:main',
        ],
    },
)
