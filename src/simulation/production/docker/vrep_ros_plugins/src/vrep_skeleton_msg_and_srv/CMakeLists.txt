cmake_minimum_required(VERSION 2.8.3)
project(vrep_skeleton_msg_and_srv)


find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

include_directories(${catkin_INCLUDE_DIRS})

# add_message_files(FILES yourMessageHere.msg)

add_service_files(FILES displayText.srv)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  LIBRARIES
  CATKIN_DEPENDS message_runtime
)
