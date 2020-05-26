cmake_minimum_required(VERSION 2.8.3)
project(controls)

if ("$ENV{COMPUTER_TYPE}" STREQUAL "onboard")

  ## Find catkin macros and libraries
  find_package(catkin REQUIRED COMPONENTS
	message_generation
	std_msgs
	nav_msgs
	geometry_msgs
	std_srvs
	tf
	pid
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
  ThrusterSpeeds.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
catkin_package(
  LIBRARIES controls
  CATKIN_DEPENDS rospy std_msgs nav_msgs geometry_msgs std_srvs tf pid
)
