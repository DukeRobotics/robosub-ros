cmake_minimum_required(VERSION 3.0.2)
project(google_drive)

## Find catkin macros and libraries
find_package(catkin REQUIRED
  rospy
  custom_msgs
  resource_retriever
)

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
catkin_package(CATKIN_DEPENDS rospy custom_msgs resource_retriever)
