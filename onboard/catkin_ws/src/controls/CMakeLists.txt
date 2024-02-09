cmake_minimum_required(VERSION 2.8.3)
project(controls)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  std_msgs
  nav_msgs
  geometry_msgs
  custom_msgs
  std_srvs
  tf
  pid
  resource_retriever
)

find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)

## Specify additional locations of header files
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
catkin_package(
  CATKIN_DEPENDS roscpp rospy roslib std_msgs nav_msgs geometry_msgs std_srvs custom_msgs tf pid resource_retriever
)

## Build C++ executables and libraries
add_executable(controls_test src/test.cpp src/controls_utils.cpp)
target_link_libraries(controls_test
  yaml-cpp
  Eigen3::Eigen
  OsqpEigen::OsqpEigen
  ${catkin_LIBRARIES}
)

set(SOURCES
  src/controls.cpp
  src/thruster_allocator.cpp
  src/pid_manager.cpp
  src/drc_pid.cpp
  src/controls_utils.cpp
)

add_executable(controls ${SOURCES})
target_link_libraries(controls
  yaml-cpp
  Eigen3::Eigen
  ${catkin_LIBRARIES}
)

target_precompile_headers(controls
  PRIVATE
    <ros/ros.h>
    <Eigen/Dense>
    <yaml-cpp/yaml.h>
)
target_precompile_headers(controls_test REUSE_FROM controls)


