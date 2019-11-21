cmake_minimum_required(VERSION 2.8.3)
project(ros_bubble_rob2)

find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS std_msgs geometry_msgs roscpp)

include_directories(${catkin_INCLUDE_DIRS})
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_executable(rosBubbleRob2 src/rosBubbleRob2.cpp) 
target_link_libraries(rosBubbleRob2 ${catkin_LIBRARIES})
add_dependencies(rosBubbleRob2 ${catkin_EXPORTED_TARGETS})

