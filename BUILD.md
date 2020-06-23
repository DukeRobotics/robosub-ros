# Building robosub-ros

We use a build script that simplifies the process of building and overlaying catkin workspaces.
 
## Executing build script 
To build our workspaces for ROS, you may execute the following command in the base directory of our repository:
```bash
./build.sh <workspace>
```
where `<workspace>` is the workspace you would like to build, either `onboard` or `landside`.

## Sourcing setup file
Once the build script has finished executing, you may go ahead and source the setup file using
```bash
source <workspace>/catkin_ws/devel/setup.bash
```
where `<workspace>` is the workspace you just built. This will also be the last line that is 
printed from the build script, so you can also just copy and execute that.

You should now be ready to use our packages and ROS.

## Cleaning build
If you ever wish to clean the build outputs from a workspace, you may change directory
into the `catkin_ws` folder located in the workspace. Once there you may execute 
```bash
catkin clean
``` 
which will remove the build, devel, and logs folders that were created when building
the workspace.
