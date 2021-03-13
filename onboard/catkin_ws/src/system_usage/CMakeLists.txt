cmake_minimum_required(VERSION 3.0.2)
project(system_usage)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  GPUtil
  custom_msgs
  psutil
  rospy
)

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
catkin_package(CATKIN_DEPENDS GPUtil custom_msgs psutil rospy)
