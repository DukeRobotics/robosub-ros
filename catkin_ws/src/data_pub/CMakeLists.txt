cmake_minimum_required(VERSION 2.8.3)
project(data_pub)

if ("$ENV{COMPUTER_TYPE}" STREQUAL "onboard")

  ## Find catkin macros and libraries
  find_package(catkin REQUIRED COMPONENTS
    rospy
    tf
    message_generation
    std_msgs
    nav_msgs
    geometry_msgs
  )

else()

  ## Find catkin macros and libraries only needed to create new messages
  find_package(catkin REQUIRED COMPONENTS
	message_generation
	std_msgs
  )

endif ("$ENV{COMPUTER_TYPE}" STREQUAL "onboard")

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  DVLRaw.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
catkin_package(
  LIBRARIES data_pub
  CATKIN_DEPENDS rospy message_runtime std_msgs nav_msgs geometry_msgs tf
  DEPENDS python-numpy python-serial
)
