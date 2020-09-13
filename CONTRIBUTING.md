# Contributing

When contributing to this repository, please first discuss the change with a CS team lead.

## Pull Request Process

This guide details how to format a new ROS package for our repository. This assumes that you have already created the package and have tested it.

1. Ensure that the package is located in the appropriate workspace. Packages that will be run onboard the robot during competition should be located in the onboard workspace. Packages that require human input or are designed to be used by humans that will not be used during an autonomous run should be located in the landside workspace.

2. Move any custom messages, services, and actions to the `custom_msgs` package located in the core workspace. You should then add them to the `CMakeLists.txt` and `package.xml` in the `custom_msgs` package as necessary.

3. Update the `CMakeLists.txt` file in your package. You should remove all commented items. Most packages will contain two commands, a `find_package` and a `catkin_package`. The `find_package` should contain all of the ROS packages needed for your project. The `catkin_package` should contain the `CATKIN_DEPENDS` tag and list all of the ROS packages needed for your project. You can look at another `CMakeLists.txt` in our repository as a guide to how yours should be formatted.

4. Update the `package.xml` file in your package. You should remove all comments. Add a description of your package to the `<description>` tag. The `<maintainer>` tag should be the Duke Robotics Club and the email should be the official Duke Robotics Club email. The `<license>` tag should be MIT. The `<depend>` tag should contain all of the ROS packages that your package requires, most likely those listed in the `find_packages` in the `CMakeLists.txt`. The `<buildtool_depend>` should contain the build tools your package requires, most likely being `catkin`. Finally, the `<exec_depend>` tag should contain the non-ROS python packages that your package needs, like numpy, and they should use the ROS indexed name. You can look at other packages in our repository for examples.

5. Create a README.md in the base folder of your package that details how to use your package, the general structure of the package, and how your package works.

6. Ensure that your code adheres to good standards and practices. Your Python code should adhere to PEP8, and any code that does not adhere to PEP8 will be noted in a comment on the PR that you make.

7. After you have opened a Pull Request, our automated builds will run for your branch, and they will be run in containers using the official dukerobotics/robosub-ros images. You will be able to view the output of each build under the checks tab on GitHub for your PR. If your build fails due to an insufficient dependency, you should add the dependency to our image using the process mentioned [below](#adding-dependencies).

8. You may merge the Pull Request once you have the sign-off of one of the CS leads, or if you do not have permission to do that, you may request one of the CS leads to merge it for you.

## Adding Dependencies

Here is the process for adding dependencies:

1. Dependencies for a package in the onboard workspace can be added to the onboard Dockerfile located [here](docker/onboard/Dockerfile). Dependencies for a package in the landside workspace can be added to the landside Dockerfile located [here](docker/landside/Dockerfile). APT packages can be added to the apt-get install command, and pip packages can be added to the pip install command. Other and new methods of package installations can be added by adding a RUN command followed by the steps in bash to install the package. For more information, please see documentation about Docker located [here](https://docs.docker.com/engine/reference/builder/).

2. Ensure that the dependencies are not located in both the onboard and landside Dockerfiles. If they are, you should refactor the dependencies that are common to both images into the the core Dockerfile located [here](docker/core/Dockerfile).

3. All of our automated builds will be run when you create/update a pull request. These automatic builds will build all the images required, specifically, it will build the core and onboard images for both arm64 and amd64 architectures, and the landside image for the amd64 architecture. It will also build the our codebase, to test whether all dependencies have been installed. You can view the status of each build by going to the checks tab on the PR.
