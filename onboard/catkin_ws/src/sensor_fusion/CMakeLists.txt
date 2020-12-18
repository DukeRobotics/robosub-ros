cmake_minimum_required(VERSION 2.8.3)
project(sensor_fusion)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  robot_localization
  nav_msgs
  sensor_msgs
  geometry_msgs
)

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
catkin_package(CATKIN_DEPENDS rospy robot_localization nav_msgs sensor_msgs geometry_msgs robosub_description)
