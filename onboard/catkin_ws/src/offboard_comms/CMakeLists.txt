cmake_minimum_required(VERSION 2.8.3)
project(offboard_comms)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rosserial_arduino
  rosserial_client
  rosserial_python
  custom_msgs
  std_msgs
  sensor_msgs
)

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
catkin_package(CATKIN_DEPENDS rosserial rosserial_arduino std_msgs custom_msgs sensor_msgs)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
