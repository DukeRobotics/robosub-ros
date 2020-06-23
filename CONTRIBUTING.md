# Contributing

When contributing to this repository, please first discuss the change with a CS team lead.

## Pull Request Process

This guide details how to format a new ROS package for our repository. This assumes that you have already created the
package and have tested it.

1. Ensure that the package is located in the appropriate workspace. Packages that will be run onboard
   the robot during competition should be located in the onboard workspace. Packages that require 
   human input or are designed to be used by humans that will not be used during an autonomous run 
   should be located in the landside workspace.
   
2. Move any custom messages, services, and actions to the `custom_msgs` package located in the core workspace. You
   should then add them to the `CMakeLists.txt` and `package.xml` in the `custom_msgs` package as necessary.
   
3. Update the `CMakeLists.txt` file in your package. You should remove all commented items. Most packages will contain
   two commands, a `find_package` and a `catkin_package`. The `find_package` should contain all of the ROS packages 
   needed for your project. The `catkin_package` should contain the `CATKIN_DEPENDS` tag and list all of the ROS
   packages needed for your project. You can look at another `CMakeLists.txt` in our repository as a guide to how yours
   should be formatted.
     
4. Update the `package.xml` file in your package. You should remove all comments. Add a description of your package to 
   the  `<description>` tag. The `<maintainer>` tag should be the Duke Robotics Club and the email should be the official
   Duke Robotics Club email. The `<license>` tag should be MIT. The `<depend>` tag should contain all of the ROS packages
   that your package requires, most likely those listed in the `find_packages` in the `CMakeLists.txt`. The
   `<buildtool_depend>` should contain the build tools your package requires, most likely being `catkin`. Finally,
   the `<exec_depend>` tag should contain the non-ROS python packages that your package needs, like numpy, and they 
   should use the ROS indexed name. You can look at other packages in our repository for examples.

5. Create a README.md in the base folder of your package that details how to use your package, the general 
   structure of the package, and how your package works.

6. Ensure that your code adheres to good standards and practices. Your Python code should adhere to PEP8, and any
   code that does not adhere to PEP8 will be noted in a comment on the PR that you make. 

7. After you have opened a Pull Request, our automated builds will run for your branch, and they will be run in
   contianers using the official dukerobotics/robosub-ros images. You will be able to view the
   output of each build under the checks tab on Github for your PR. If your build fails due to an insufficient dependency,
   you should add the dependency to our image using the process mentioned 
   [here](https://github.com/DukeRobotics/robosub-ros-docker/blob/master/CONTRIBUTING.md). 

7. You may merge the Pull Request in once you have the sign-off of one of the CS leads, or if you 
   do not have permission to do that, you may request one of the CS leads to merge it for you.
   