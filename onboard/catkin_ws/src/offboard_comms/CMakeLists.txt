cmake_minimum_required(VERSION 2.8.3)
project(offboard_comms)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rosserial_arduino
  rosserial_client
  rosserial_python
  message_generation
  std_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  ThrusterSpeeds.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  SetServo.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
catkin_package(
  LIBRARIES offboard_comms
  CATKIN_DEPENDS rosserial rosserial_arduino message_runtime std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
