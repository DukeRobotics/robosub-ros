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
  ControlTypes.msg
  CVObject.msg
  DVLRaw.msg
  HydrophoneSet.msg
  Memory.msg
  PIDAxesInfo.msg
  PIDDerivativeType.msg
  PIDGain.msg
  PIDGains.msg
  PIDInfo.msg
  PIDTerms.msg
  PWMAllocs.msg
  RemoteLaunchInfo.msg
  RunningNode.msg
  ServoAngleArray.msg
  SimObject.msg
  SimObjectArray.msg
  SonarSweepRequest.msg
  SonarSweepResponse.msg
  SystemUsage.msg
  TaskUpdate.msg
  ThrusterAllocs.msg
)

add_service_files(
  FILES
  ConnectDepthAICamera.srv
  ConnectUSBCamera.srv
  EnableModel.srv
  GetRunningNodes.srv
  SetControlTypes.srv
  SetPIDGains.srv
  SetPowerScaleFactor.srv
  SetServo.srv
  SetStaticPower.srv
  StartLaunch.srv
  StopLaunch.srv
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
