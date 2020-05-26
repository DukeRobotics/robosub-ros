cmake_minimum_required(VERSION 2.8.3)
project(sensor_fusion)

if ("$ENV{COMPUTER_TYPE}" STREQUAL "onboard")

  ## Find catkin macros and libraries
  find_package(catkin REQUIRED COMPONENTS
    rospy
    robot_localization
    nav_msgs
    sensor_msgs
    geometry_msgs
  )

else()

  find_package(catkin REQUIRED)

endif ("$ENV{COMPUTER_TYPE}" STREQUAL "onboard")

## The catkin_package macro generates cmake config files for your package
catkin_package(
  LIBRARIES sensor_fusion
  CATKIN_DEPENDS rospy robot_localization nav_msgs sensor_msgs geometry_msgs robosub_description
)
