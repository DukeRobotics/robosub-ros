cmake_minimum_required(VERSION 2.8.3)
project(static_transforms)

if ($ENV{COMPUTER_TYPE}==onboard)

  find_package(catkin REQUIRED COMPONENTS
    tf2_ros
  )

  ## The catkin_package macro generates cmake config files for your package
  ## Declare things to be passed to dependent projects
  catkin_package(
    LIBRARIES static_transforms
    CATKIN_DEPENDS tf2_ros
  )

endif ($ENV{COMPUTER_TYPE}==onboard)
