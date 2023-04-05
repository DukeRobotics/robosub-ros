cmake_minimum_required(VERSION 2.8.3)
project(sonar)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  actionlib
  custom_msgs
  tf
  geometry_msgs
  cv
)

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
catkin_package(CATKIN_DEPENDS 
  rospy
  actionlib
  custom_msgs
  tf
  geometry_msgs
  cv
)
