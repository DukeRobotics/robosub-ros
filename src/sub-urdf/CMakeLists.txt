<<<<<<< HEAD
cmake_minimum_required(VERSION 2.8.3)

project(sub-urdf)

find_package(catkin REQUIRED)

catkin_package()

find_package(roslaunch)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
=======
cmake_minimum_required(VERSION 2.8.3)

project(sub-urdf)

find_package(catkin REQUIRED)

catkin_package()

find_package(roslaunch)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
>>>>>>> 2eb5e5d35e92279d98ff38c4eb0d99b03ab7adc1
