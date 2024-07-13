cmake_minimum_required(VERSION 3.0.2)
project(yolov7)

find_package(catkin REQUIRED COMPONENTS rospy)

catkin_python_setup()

file(GLOB PYTHON_SCRITPS_DIR
	"src/*.py"
)

catkin_package(	CATKIN_DEPENDS )

catkin_install_python(
	PROGRAMS ${PYTHON_SCRITPS_DIR}
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)