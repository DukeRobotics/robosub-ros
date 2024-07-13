cmake_minimum_required(VERSION 3.0.2)
project(image_transport_py)

find_package(
	catkin
	REQUIRED
	COMPONENTS
	rospy
	sensor_msgs
	std_msgs
)

catkin_python_setup()

file(GLOB PYTHON_SCRITPS_DIR
	"src/*.py"
)

catkin_package(
	CATKIN_DEPENDS
)

catkin_install_python(
	PROGRAMS ${PYTHON_SCRITPS_DIR}
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)