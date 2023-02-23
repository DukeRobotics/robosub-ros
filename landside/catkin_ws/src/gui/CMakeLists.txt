cmake_minimum_required(VERSION 3.0.2)
project(gui)

find_package(catkin REQUIRED COMPONENTS
  rospy
  rqt_gui
  rqt_gui_py
  geometry_msgs
  std_srvs
  nav_msgs
  resource_retriever
  rosservice
  tf
  dynamic_reconfigure
  custom_msgs
  robot_localization
)

catkin_python_setup()
catkin_package()

install(DIRECTORY
  resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES
  plugins/offboard_plugin.xml
  plugins/sensor_plugin.xml
  plugins/controls_plugin.xml
  plugins/launch_plugin.xml
  plugins/rosbag_plugin.xml
  plugins/system_usage_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
) 
