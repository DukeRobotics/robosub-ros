cmake_minimum_required(VERSION 2.8.3)
project(cv)


## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    custom_msgs
	sensor_msgs
	geometry_msgs
	cv_bridge
	rospy
	resource_retriever
	sonar
)

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
catkin_package(CATKIN_DEPENDS rospy sensor_msgs custom_msgs geometry_msgs cv_bridge resource_retriever sonar)


