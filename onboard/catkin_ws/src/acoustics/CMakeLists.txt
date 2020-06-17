cmake_minimum_required(VERSION 2.8.3)
project(acoustics)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  roscpp
  rospy
  std_msgs
  geometry_msgs
)


add_action_files(
  FILES
  processing.action
)

generate_messages(DEPENDENCIES actionlib_msgs std_msgs geometry_msgs)


catkin_package(
   CATKIN_DEPENDS actionlib actionlib_msgs message_runtime roscpp rospy std_msgs geometry_msgs
)
