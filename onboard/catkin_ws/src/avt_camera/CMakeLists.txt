cmake_minimum_required(VERSION 2.8.3)
project(avt_camera)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  cv_bridge
  camera_info_manager_py
)

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
catkin_package(CATKIN_DEPENDS rospy sensor_msgs cv_bridge camera_info_manager_py)
