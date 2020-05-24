cmake_minimum_required(VERSION 2.8.3)
project(execute)

## Find catkin macros and libraries
find_package(catkin REQUIRED)

catkin_package(
    LIBRARIES execute
    CATKIN_DEPENDS data_pub controls static_transforms motion_planning sensor_fusion
)
