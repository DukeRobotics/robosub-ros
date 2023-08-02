cmake_minimum_required(VERSION 2.8.3)
project(custom_msgs)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
  actionlib_msgs
)

add_message_files(
  FILES
  DVLRaw.msg
  HydrophoneSet.msg
  ThrusterSpeeds.msg
  ServoAngleArray.msg
  CVObject.msg
  Memory.msg
  SystemUsage.msg
  SimObject.msg
  SimObjectArray.msg
  RemoteLaunchInfo.msg
  RunningNode.msg
)

add_service_files(
  FILES
  ConnectDepthAICamera.srv
  ConnectUSBCamera.srv
  EnableModel.srv
  SetServo.srv
  StartLaunch.srv
  StopLaunch.srv
  GetRunningNodes.srv
)

add_action_files(
  FILES
  AcousticsProcessing.action
  AcousticsGuess.action
  AcousticsWrapper.action
  Saleae.action
  AcousticsData.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
)

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs actionlib_msgs
)
