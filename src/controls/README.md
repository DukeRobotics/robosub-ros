# Controls


These are the instructions to intialize and test our controls algorithm. Currently, our goal is to take input from various topics and move the robot based on desired global position or desired power. By default, we publish thruster allocations on a range [-128, 127] that are readily available for the Arduino. We can also publish values on a range [-1, 1] for use by the simulation.

Thruster information is read from `cthulhu.config`, which is written in YAML and can be easily updated if thrusters are added or moved. The xyz positions are in meters, and the rpy (roll, pitch, yaw) rotations are in degrees.

**Note**: The ordering of the thruster power outputs is determined by the order in which they appear the config file.


## Setup

1. Make sure that you have the latest controls code (from controls or master branch), your catkin workspace has been built, and your terminal has been sourced correctly.
2. SSH into the docker container, either on the robot or [on your local computer](https://github.com/DukeRobotics/documentation/blob/master/docker/README.md), and maybe [configured for simulation](https://github.com/DukeRobotics/robosub-ros/blob/master/simulation/README.md).
3. Initialize `roscore` and run the following in the terminal to start the PID Loops:

    ```roslaunch controls controls.launch &```

## Testing Outputs

To enable non-zero thruster output, the following command should be used:

    ```rosservice call /enable_controls true```

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
roslaunch controls controls.launch mode:=sim
```

`test_state_publisher.py` is where we specify the desired state of the robot. Alternatively, you can publish to either of the desired state topics directly using your own code. The second command launches the entire controls node in simulation mode.


## Topics

### Listening

Desired State Topics:

  - ```controls/desired_pose```
    + A point and quaternion representing the robot's desired global xyz position and rpy orientation
    + Type: geometry_msgs/Pose
  - ```controls/desired_twist_power```
    + A twist with values [-1,1] corresponding to relative linear or angular velocity. 1 is full speed in a positive direction, -1 is full speed in the negative direction.
    + 0 values are interpreted as axes to stabilize on. Stabilization is performed by using position control rather than velocity control on 0'ed axes to mitigate drift effects or other unwanted perturbations.
    + For instance, a twist with values [1,0,0,0,0,0] will result in full speed in the positive local x-direction and stabilization along all other axes.
    + Type: geometry_msgs/Twist

Current State Topics:

  - ```/state``` 
    + The current state of the actual robot, in global position, orientation, linear velocity, and angular velocity
    + Type: nav_msgs/Odometry
  - ```/sim/pose```
    + Current position and orientation of the robot in simulation
    + Type: geometry_msgs/PoseStamped
  - ```/sim/dvl```
    + Current linear and angular velocities of the robot in simulation
    + Type: geometry_msgs/TwistStamped 

### Publishing

We can choose to publish to either of these topics by changing the `mode` argument in the `controls.launch` file. It defaults to `robot`, but can also be `sim`.

  - ```/offboard/thruster_speeds```
    + An array of 8 8-bit integers [-128,127] describing the allocation of the thrusters sent to the Arduino
    + Type: controls.msg/ThrusterSpeeds
  - ```/sim/move```
    + An array of 8 floats [-1,1] describing the allocation of the thrusters sent to the simulation
    + Type: std_msgs/Float32MultiArray

### Important Notes

Only the most recently updated Desired State Topic will be used in movement. Therefore any updates will override the current movement of the robot. Controls will warn you if more than one Desired State Topic is being published to at any given time to prevent such issues. Also, if Controls stops receiving Desired State messages at a high enough rate [TBD], it will warn you and will output zero power for safety purposes.

The controls algorithm will output all 0's unless it is enabled with a call to rosservice as detailed in the setup. This is intended to act as an emergency stop that cuts off all power to the thrusters if necessary.


## How It Works (Structure and Flow)

### Structure

This package contains the following custom ROS nodes:

* `state_republisher`
* `desired_state`
* `thruster_controls`

This package has the following launch files:

* `controls.launch` is the entrypoint to the package. It takes in a `mode` argument to indicate whether we are in `robot` (default) or `sim`ulation mode. It includes the `pid.launch` file to launch the PID for position loops. It then starts the three nodes above.
* `pid.launch` spins up six [ROS PID](http://wiki.ros.org/pid) nodes for position control on x, y, z, roll, pitch, and yaw. It defines the PID parameters at the top, depending on the `mode` argument passed in.

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

### File-by-file description

#### Launch and message

Described above.

#### Scripts

* `state_republisher.py` - listens to Current State Topics and republishes relevant components to their own `/controls/state/...` topics for use in all of the other parts of this controls package.
* `desired_state.py` - listens to Desired State Topics, warning the user via the console if none or more than one are received, and outputting setpoints to PID loops if desiring position, or powers directly to `thruster_controls` for axes that are velocity/power-controlled.
* `thruster_controls.py` - listens to control efforts from PID or `desired_state`, uses an instance of `ThrusterManager` to calculate thruster allocations from the them, scales outputs so that the maximum is 1 or -1 in any direction, and publishes to Arudino or simulation movement topics.
* `thruster_manager.py` - Defines the `ThrusterManager` class, which reads in config file, creates `Thruster` array from it, and has math for calculating thruster allocations.
* `thruster.py` - Defines the `Thruster` class, which takes in a position and orientation of a thruster relative to the center of the robot, and then calculates and stores the force and torque it exerts on the robot.
* `drc_math.py` - A library of mathematical functions for controls. (DRC stands for Duke Robotics Club.)
* `cthulhu.config` - The config file for Cthulhu, our RoboSub 2019 and RoboSub 2020 sub. Thrusters are named, given a type, their xyz position is measured in meters from the center of the robot, and their rpy orientation is an offset from robot frame.