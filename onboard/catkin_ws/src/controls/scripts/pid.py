#!/usr/bin/env python3

"""
TODO: Implement a PID controller
The PID controller should be a class that is initialized with the PID gains (Kp, Ki, Kd, and feedforward).

There should be a method that takes in the error value and delta time and returns the control effort. The feedfoward
term should be added to the control effort just before returning. The final returned value must always be
between -1 and 1 (inclusive).

There should be a method that can reset the PID controller; this would be called when a new setpoint is given.

It should be possible for the PID gains to be changed at any time, even while the controller is running.

Context: One instance of this class will be created per axis (x, y, z, roll, pitch, yaw) per control type
(position, velocity). The first method described above will be called every time /state is published. The second
method will be called every time a new setpoint is published to /desired_pose or /desired_twist. The PID gains will
be updated every time a service is called to change them (this service will be advertised by separate node).

IMPORTANT: Make sure to look at the ROS PID implementation at (https://bitbucket.org/AndyZe/pid/src). Pay special
attention to the filter and determine if it is necessary for our implementation. See https://wiki.ros.org/pid for more
information on the filter and the package in general.
"""