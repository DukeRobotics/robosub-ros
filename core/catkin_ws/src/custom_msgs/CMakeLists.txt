cmake_minimum_required(VERSION 2.8.3)
project(custom_msgs)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

add_message_files(
  FILES
  DVLRaw.msg
  ThrusterSpeeds.msg
  ServoAngle.msg
  CVObject.msg
)

add_service_files(
  FILES
  SetServo.srv
  EnableModel.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)
