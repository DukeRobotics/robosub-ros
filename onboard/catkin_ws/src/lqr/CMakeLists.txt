cmake_minimum_required(VERSION 2.8.3)
project(lqr)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  nav_msgs
  geometry_msgs
  custom_msgs
  std_srvs
  tf
  pid
)

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
catkin_package(CATKIN_DEPENDS rospy std_msgs nav_msgs geometry_msgs std_srvs custom_msgs tf pid)
