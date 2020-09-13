# Contributing

When contributing to this repository, please first discuss the change you wish to make with either the CS or electronics team leads before making a change.

## Pull Request Process

The vast majority of PRs targeting this repository should be about adding new and necessary dependencies for new software packages.

Here is the process for adding dependencies:

1. Dependencies for a package in the onboard workspace can be added to the onboard Dockerfile located [here](https://github.com/DukeRobotics/robosub-ros-docker/blob/master/onboard/Dockerfile). Dependencies for a package in the landside workspace can be added to the landside Dockerfile located [here](https://github.com/DukeRobotics/robosub-ros-docker/blob/master/landside/Dockerfile). APT packages can be added to the apt-get install command, and pip packages can be added to the pip install command. Other and new methods of package installations can be added by adding a RUN command followed by the steps in bash to install the package. For more information, please see documentation about Docker located [here](https://docs.docker.com/engine/reference/builder/).

2. Ensure that the dependencies are not located in both the onboard and landside Dockerfiles. If they are, you should refactor the dependencies that are common to both images into the the core Dockerfile located [here](https://github.com/DukeRobotics/robosub-ros-docker/blob/master/core/Dockerfile).

3. All of our automated builds will be run when you create a new pull request. These automatic builds will build all the images required, specifically, it will build the core and onboard images for both arm64 and amd64 architectures, and the landside image for the amd64 architecture. It will also build the master branch of our codebase, located [here](https://github.com/DukeRobotics/robosub-ros), to test whether all dependencies have been installed on the master branch. You can view the status of each build by going to the checks tab on the PR.

4. Once the automated builds have passed, you should test whether your images contains the dependencies for the branch in the code repository that contains your packages. You can do this by creating a comment on the PR you opened in this docker repository with the format `/docker-test <branch_name>` where `<branch_name>` is the branch in the code base that you would like to test against. For instance, to test against branch `cv` of the codebase, you would comment `/docker-test cv`. Please ensure that there is no other text in the comment, and to test multiple branches, you will have to make multiple comments. An example can be found [here](https://github.com/DukeRobotics/robosub-ros-docker/pull/48).

5. The GitHub bot will then add reactions to your comment, indicating that it has seen the request and has started the builds. Once the builds are done, a comment will be created on the PR by the GitHub bot that will indicate whether all builds/tests passed.

6. You may merge the Pull Request in once you have the sign-off of one of the team leads, or if you do not have permission to do that, you may request the leads to merge it for you.
