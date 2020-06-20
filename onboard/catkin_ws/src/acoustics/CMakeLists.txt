cmake_minimum_required(VERSION 2.8.3)
project(acoustics)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
find_package(catkin REQUIRED COMPONENTS
  actionlib
  rospy
  custom_msgs
)


catkin_package(
   CATKIN_DEPENDS rospy custom_msgs actionlib
)
