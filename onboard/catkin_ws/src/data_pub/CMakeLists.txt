cmake_minimum_required(VERSION 2.8.3)
project(data_pub)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  tf
  custom_msgs
  std_msgs
  nav_msgs
  geometry_msgs
)

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
catkin_package(CATKIN_DEPENDS rospy custom_msgs std_msgs nav_msgs geometry_msgs tf)
