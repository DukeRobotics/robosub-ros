cmake_minimum_required(VERSION 2.8.3)
project(static_transforms)

find_package(catkin REQUIRED COMPONENTS
  tf2_ros
)

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
catkin_package(CATKIN_DEPENDS tf2_ros)
