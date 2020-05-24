cmake_minimum_required(VERSION 2.8.3)
project(joystick)

if ("$ENV{COMPUTER_TYPE}" STREQUAL "landside")

  ## Find catkin macros and libraries
  find_package(catkin REQUIRED COMPONENTS
    rospy
    sensor_msgs
    geometry_msgs
    joy
  )

else()

  find_package(catkin REQUIRED)

endif ("$ENV{COMPUTER_TYPE}" STREQUAL "landside")

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
catkin_package(
  LIBRARIES joystick
  CATKIN_DEPENDS rospy sensor_msgs geometry_msgs joy
)
