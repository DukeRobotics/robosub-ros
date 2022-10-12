cmake_minimum_required(VERSION 2.8.3)
project(cv_control_panel)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  image_view
  image_publisher
  rosbag
  rospy
  sensor_msgs
  std_msgs
  cv_bridge
  resource_retriever
)

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
catkin_package(CATKIN_DEPENDS image_view image_publisher rosbag rospy sensor_msgs std_msgs cv_bridge resource_retriever)
