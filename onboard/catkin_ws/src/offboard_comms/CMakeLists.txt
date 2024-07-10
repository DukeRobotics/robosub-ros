cmake_minimum_required(VERSION 2.8.3)
project(offboard_comms)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  rosserial_arduino
  rosserial_client
  rosserial_python
  custom_msgs
  std_msgs
  sensor_msgs
  geometry_msgs
)

include(ExternalProject)
ExternalProject_Add(serial_external
  GIT_REPOSITORY https://github.com/wjwwood/serial.git
  GIT_TAG main
  PREFIX ${CMAKE_CURRENT_BINARY_DIR}/serial
  # INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/serial
  # CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_BINARY_DIR}/serial/install
  BUILD_COMMAND ${CMAKE_COMMAND} --build . --target install
)

ExternalProject_Get_Property(serial_external install_dir)
# set(SERIAL_INCLUDE_DIRS ${INSTALL_DIR}/include)
# set(SERIAL_LIBRARIES ${INSTALL_DIR}/lib/libserial.so)

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
catkin_package(CATKIN_DEPENDS roscpp roslib rosserial rosserial_arduino std_msgs custom_msgs sensor_msgs geometry_msgs)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${install_dir}/include
)

link_directories(
  ${install_dir}/lib
)

add_executable(thrusters src/thrusters.cpp src/serial_thrusters.cpp src/nonlinear_thrusters.cpp)
add_dependencies(thrusters serial_external)

target_link_libraries(thrusters ${catkin_LIBRARIES} serial)

