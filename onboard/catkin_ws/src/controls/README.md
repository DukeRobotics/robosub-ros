# Controls


These are the instructions to initialize and test our controls algorithm. We take input from various topics and move the robot based on desired global position or desired power. By default, we publish thruster power outputs (thruster allocations) on a range [-128, 127] for the Arduino. We can also publish values on a range [-1, 1] for the simulation.

Thruster information is read from `*.config` files, which are written in YAML. The ordering of the thruster allocations is determined by the order in which the thrusters appear the config file.

`controls.launch` takes in a `sim` argument to indicate whether we are publishing for the simulation or the Arduino.

Only the most recently updated Desired State Topic will be used in movement. Therefore any updates will override the current movement of the robot. Controls will warn you if more than one Desired State Topic is being published to at any given time to prevent such issues. If Controls stops receiving Desired State messages at a high enough rate (at the moment, 10 Hz), it will warn you and will output zero power for safety purposes.

The controls algorithm will output all 0's unless it is enabled with a call to rosservice as detailed in the setup. Sending the disable call to the service acts as an emergency stop that cuts off all power to the thrusters.


## Setup

1. Make sure that you have the latest controls code (from controls or master branch), your catkin workspace has been built, and your terminal has been sourced correctly.
2. SSH into the docker container, either on the robot or [on your local computer](https://github.com/DukeRobotics/documentation/blob/master/docker/README.md), and maybe [configured for simulation](https://github.com/DukeRobotics/robosub-ros/blob/master/simulation/README.md).
3. Initialize `roscore` and run the following in the terminal to start the PID Loops:

```
roslaunch controls controls.launch &
```

## Testing Outputs

To enable non-zero thruster output, the following command should be used:
```
rosservice call /enable_controls true
```
To test the outputs of the PID Loops, edit the values in `test_state_publisher.py` to whatever current and desired state you wish to test. Then, run the following:

```
rosrun controls test_state_publisher.py &
```

To check direct PID Outputs:

```
rostopic echo /control_effort/<var>
```

Where &lt;var&gt; is x, y, z, roll, pitch, or yaw. To check final thruster allocations:

```
rostopic echo /offboard_comms/ThrusterSpeeds
```


## Testing with Simulation

To get the initialize the simulation, follow the instructions in the simulation directory.

Once the simulation is running, execute:

```
rosrun controls test_state_publisher.py &
roslaunch controls controls.launch sim:=true
```

`test_state_publisher.py` is where we specify the desired state of the robot. Alternatively, you can publish to any of the desired state topics directly using your own code. The second command launches the entire controls node in simulation mode.


## Topics

### Listening

Desired State Topics:

  - ```controls/desired_pose```
    + A point and quaternion representing the robot's desired global xyz position and rpy orientation.
    + Type: geometry_msgs/Pose
  - ```controls/desired_power```
    +  A twist with values [-1,1] (TBD) corresponding to global linear and angular velocities.
    + Type: geometry_msgs/Twist

  - ```controls/desired_twist```
    + A twist with values [-1,1] corresponding to relative linear and angular velocities. 1 is full speed in a positive direction, -1 is full speed in the negative direction.
    + This option completely ignores all PID loops, and offers no stabilization. It is mainly for use with joysticks.
    + Type: geometry_msgs/Twist

Current State Topics:

  - ```/state``` 
    + The current state of the robot or simulated robot, in global position, orientation, linear velocity, and angular velocity
    + Type: nav_msgs/Odometry

### Publishing

We can choose to publish to either of these topics by changing the `sim` argument in the `controls.launch` file. It defaults to `false` for the Arduino, but can also be `true` for the simulation.

  - ```/offboard/thruster_speeds```
    + An array of 8 8-bit integers [-128,127] describing the allocation of the thrusters sent to the Arduino
    + Type: controls.msg/ThrusterSpeeds
  - ```/sim/move```
    + An array of 8 floats [-1,1] describing the allocation of the thrusters sent to the simulation
    + Type: std_msgs/Float32MultiArray


## How It Works

### Structure

This package contains the following custom ROS nodes:

* `state_republisher`
* `desired_state`
* `thruster_controls`

This package has the following launch files:

* `controls.launch` is the entrypoint to the package. It takes in a `sim` argument to indicate whether we are publishing for the simulation or the Arduino. It includes the `pid.launch` file to launch the PID for position loops. It then starts the three nodes above.
* `position_pid.launch` spins up six [ROS PID](http://wiki.ros.org/pid) nodes for position control on x, y, z, roll, pitch, and yaw. It defines the PID parameters at the top, depending on the `sim` argument passed in.
* `velocity_pid.launch` spins up six [ROS PID](http://wiki.ros.org/pid) nodes for velocity control on x, y, z, roll, pitch, and yaw. It defines the PID parameters at the top, depending on the `sim` argument passed in.

This package also defines a new custom message type, `ThrusterSpeeds`, which is the same type as in the package for controlling the Arduino.

### Flow

```
    Current State Topics        Desired State Topics
            |                            |
            |                            |
            v                            v
    state_republisher    ---->    desired_state
                    \             /
/controls/state/...  \           /   /controls/.../setpoint
                      \         /
                          PID
                           |
                           |   /control_effort/...
                           v
                    thruster_controls
                           |
                           |   Publishing topics
                           v
                  Arduino or Simulation

```

### PID Flow

This package uses nested PID Loops. When using Position Control, the desired state input is used as the setpoint for the position loop and the output of the position loops is used as a setpoint for the velocity loops. When using Velocity Control, the position loop is bypassed and the desired state input is used as a setpoint for the velocity loops. When using Power Control both of the PID loops are bypassed and the input is directly published to thruster_controls.
```

                      Velocity Control
      +-----------------------------------------------+
      |                                               |
      |                                               v
desired_state ---------------> position_pid ---> velocity_pid ---> thruster_controls
              Position Control
```


### Configuration

Our configuration files are written in YAML and terminate in `.config`. As of now, they only contain thruster information. Thrusters are named and given a type. A thruster's xyz position is measured in meters from the robot's center of mass, and measurements are done in robot frame. A thruster's rpy (roll, pitch, yaw) orientation is measured using extrinsic [Euler angles](https://en.wikipedia.org/wiki/Euler_angles) in degrees as an offset from robot frame. Thrusters can also be flipped. **This should be the only place in the entire codebase where flipping of thruster allocations occurs.**

Robot frame is defined as the positive x-axis pointing in the "forward" direction of movement, z-axis pointing up, and y-axis follows the right hand rule.

A thruster's starting orientation when aligned with robot frame is defined as the positive x-axis pointing in the "forward" direction of the thruster's movement, z-axis pointing up, and y-axis following the right hand rule. Note that a roll in the thruster orientation theoretically has no effect on its direction of thrust, as it just rotates about the axis of thrust. After accounting for flipping, sending a positive power to a thruster should make it move towards its "forward" or +x direction. The ordering of the thruster power outputs is determined by the order in which the thrusters appear the config file.

`cthulhu.config` is the config file for Cthulhu, our RoboSub 2019 and RoboSub 2020 robot.


### Scripts

* `state_republisher.py` - listens to Current State Topics and republishes relevant components to their own `/controls/state/...` topics for use in all of the other parts of this controls package.
* `desired_state.py` - listens to Desired State Topics, warns the user via the console if none or more than one are received, and outputs setpoints to PID loops if desiring position or velocirty, or outputs powers directly to `thruster_controls`.
* `thruster_controls.py` - listens to control efforts from PID or `desired_state`, uses an instance of `ThrusterManager` to calculate thruster allocations from the them, scales outputs so that the maximum is 1 or -1 in any direction, and publishes to Arduino or simulation movement topics.
* `thruster_manager.py` - Defines the `ThrusterManager` class, which reads in config file, creates `Thruster` array from it, and has math for calculating thruster allocations.
* `thruster.py` - Defines the `Thruster` class, which takes in a position and orientation of a thruster relative to the center of the robot, and then calculates and stores the force and torque it exerts on the robot.
* `controls_utils.py` - A library of mathematical and helper functions for controls.