cmake_minimum_required(VERSION 2.8.3)
project(data_pub)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  tf
  message_generation
  std_msgs
  nav_msgs
  geometry_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  DVLRaw.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
catkin_package(
  LIBRARIES data_pub
  CATKIN_DEPENDS rospy message_runtime std_msgs nav_msgs geometry_msgs tf
)
