cmake_minimum_required(VERSION 2.8.3)
project(controls)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  nav_msgs
  geometry_msgs
  custom_msgs
  std_srvs
  tf
  pid
  resource_retriever
)

find_package(Eigen3 REQUIRED)

## Specify additional locations of header files
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs nav_msgs geometry_msgs std_srvs custom_msgs tf pid resource_retriever
  DEPENDS Eigen3
)

## Build C++ executables and libraries
add_executable(test src/test.cpp)
target_link_libraries(test yaml-cpp ${catkin_LIBRARIES})

set(SOURCES
  src/controls.cpp
  src/thruster_allocator.cpp
)

add_executable(controls ${SOURCES})
target_link_libraries(controls yaml-cpp Eigen3::Eigen ${catkin_LIBRARIES})


