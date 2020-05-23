cmake_minimum_required(VERSION 2.8.3)
project(joystick)

if ($ENV{COMPUTER_TYPE}==landside)

  ## Find catkin macros and libraries
  find_package(catkin REQUIRED COMPONENTS
    rospy
    sensor_msgs
    geometry_msgs
    joy
  )

  ## The catkin_package macro generates cmake config files for your package
  ## Declare things to be passed to dependent projects
  catkin_package(
    LIBRARIES joystick
    CATKIN_DEPENDS rospy sensor_msgs geometry_msgs joy
    DEPENDS python-yaml
  )

endif($ENV{COMPUTER_TYPE}==landside)
