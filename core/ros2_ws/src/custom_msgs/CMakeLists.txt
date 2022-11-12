cmake_minimum_required(VERSION 3.8)
project(custom_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(actionlib_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DVLRaw.msg"
  "msg/HydrophoneSet.msg"
  "msg/ThrusterSpeeds.msg"
  "msg/ServoAngleArray.msg"
  "msg/CVObject.msg"
  "msg/Memory.msg"
  "msg/SystemUsage.msg"
  "msg/SimObject.msg"
  "msg/SimObjectArray.msg"
  "srv/SetServo.srv"
  "srv/EnableModel.srv"
  "srv/StartLaunch.srv"
  "srv/StopLaunch.srv"
  "action/AcousticsProcessing.action"
  "action/AcousticsGuess.action"  
  "action/AcousticsWrapper.action"
  "action/Saleae.action"
  "action/AcousticsData.action"
  DEPENDENCIES std_msgs geometry_msgs actionlib_msgs
)

ament_package()
