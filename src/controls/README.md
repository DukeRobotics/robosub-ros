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


## How It Works (Structure and Flow)

This package contains three ROS nodes:

* 