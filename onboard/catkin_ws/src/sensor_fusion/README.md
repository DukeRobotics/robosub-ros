# Sensor Fusion

This package mainly uses ROS's [robot_localization](http://wiki.ros.org/robot_localization) package, which "Provides nonlinear state estimation through sensor fusion of an arbitrary number of sensors."

It also uses the robot_description URDF file to publish the robot model.

It also contains a node to feed sensor data from the simulation into sensor fusion for local testing (without running the robot). Enable this by passing `sim:=true` when launching the `fuse.launch` file.
