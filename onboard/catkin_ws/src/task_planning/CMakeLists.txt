cmake_minimum_required(VERSION 2.8.3)
project(task_planning)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  nav_msgs
  geometry_msgs
  custom_msgs
  smach
  tf
  tf2
)

## The catkin_package macro generates cmake config files for your package
catkin_package(CATKIN_DEPENDS std_msgs nav_msgs geometry_msgs custom_msgs smach tf tf2)
