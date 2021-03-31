cmake_minimum_required(VERSION 2.8.3)
project(system_utils)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  custom_msgs
  rospy
)

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
catkin_package(CATKIN_DEPENDS custom_msgs rospy)
