cmake_minimum_required(VERSION 2.8.3)
project(sonar)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
<<<<<<< HEAD
  actionlib
  custom_msgs
  tf
  geometry_msgs
=======
  std_msgs
  sensor_msgs
  custom_msgs
  tf
  geometry_msgs
  cv_bridge
>>>>>>> e67322cb2253ff0f95ae4fa5a4b9dca569a7adbd
)

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
catkin_package(CATKIN_DEPENDS 
  rospy
<<<<<<< HEAD
  actionlib
  custom_msgs
  tf
  geometry_msgs
=======
  std_msgs
  sensor_msgs
  custom_msgs
  tf
  geometry_msgs
  cv_bridge
>>>>>>> e67322cb2253ff0f95ae4fa5a4b9dca569a7adbd
)
