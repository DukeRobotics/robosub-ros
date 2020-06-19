cmake_minimum_required(VERSION 2.8.3)
project(cv)


## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    message_generation
    custom_msgs
	sensor_msgs
)

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
catkin_package(CATKIN_DEPENDS rospy message_runtime sensor_msgs custom_msgs message_generation)


