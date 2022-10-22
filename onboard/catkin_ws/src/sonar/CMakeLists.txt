cmake_minimum_required(VERSION 2.8.3)
project(sonar)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  actionlib_msgs
  actionlib
  genmsg
)

add_action_files(
  DIRECTORY action
  FILES sweep.action
)

generate_messages(DEPENDENCIES 
  std_msgs
  actionlib_msgs
)

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
catkin_package(CATKIN_DEPENDS 
  rospy
  actionlib_msgs
)
