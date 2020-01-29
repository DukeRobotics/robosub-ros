# Controls


These are the instructions to intialize and test our controls algorithm. Currently, our goal is to take input from various topics and move the robot based on desired global position or desired local/global velocity. By default, we publish thruster allocations on a range [-128, 127] that are readily available for the arduino. We can also publish values on a range [-1, 1] for use by the simulation.

Thruster information is read from `cthulhu.config`, which is written in YAML and can be easily updated if thrusters are added or moved. The xyz positions are in meters, and the rpy rotations are in degrees.

**Note**: The ordering of the thruster power outputs is determined by the order in which they appear the config file.

## Setup

1. Make sure that you've pulled the latest version of the controls branch.
2. On your local computer, cd to the git repo (probably a directory named "robosub-ros").
3. On your local computer, run the docker container with the following command:

    `docker run -td -p 2200:2200 --mount type=bind,source=$PWD/src,target=/home/duke/dev/robosub-ros/catkin_ws/src  dukerobotics/robosub-ros`

4. SSH into the docker container
5. In the docker container, run the following command:

    `. ~/dev/robosub-ros/catkin_ws/src/controls/build_node.sh`

    which sources, builds the catkin workspace, and creates packages.
6. Initialize roscore and paste the following in the terminal to start the PID Loops:

    `roslaunch controls controls.launch &`

    ROS will warn you if nothing is being published to the desired or current state topics once the PID loops are launched.

## Testing

To test the outputs of the PID Loops, edit the values in `test_state_publisher.py` to whatever current and desired state you wish to test. Then, run the following:

`
rosrun controls test_state_publisher.py &
`

To check direct PID Outputs:

`
rostopic echo /control_effort/<var>
`

Where &lt;var&gt; is x, y, z, roll, pitch, or yaw. To check final thruster allocations:

`
rostopic echo /offboard_comms/ThrusterSpeeds
`


## Topics

### Listening

Desired State Topics:

  - `controls/desired_pose_global`
    + A point and quaternion representing the robot's desired global xyz position and rpy orientation
    + Type: geometry_msgs/Pose
  - `controls/desired_twist_local`
    + 2 vectors representing the robot's desired xyz and rpy velocities in the local frame
    + Type: geometry_msgs/Twist
  - `controls/desired_twist_global`
    + 2 vectors representing the robot's desired xyz and rpy velocities in the global frame
    + Type: geometry_msgs/Twist

Current State Topics:

  - `/state`
    + The current state of the actual robot, in global position, orientation, linear velocity, and angular velocity
    + Type: nav_msgs/Odometry
  - `/sim/pose`
    + Current position and orientation of the robot in simulation
    + Type: geometry_msgs/PoseStamped
  - `/sim/dvl`
    + Current linear and angular velocities of the robot in simulation
    + Type: geometry_msgs/TwistStamped

### Publishing

We can choose to publish to either of these topics:

  - `/sim/move`
    + An array of 8 floats [-1,1] describing the allocation of the thrusters sent to the simulation
    + Type: std_msgs/Float32MultiArray
  - `/offboard_comms/ThrusterSpeeds`
    + An array of 8 8-bit integers [-128,127] describing the allocation of the thrusters sent to the arduino
    + Type: std_msgs/Int8MultiArray

### Important Notes

Only the most recently updated Desired State topic will be used in movement. Therefore any updates will override the current movement of the robot. Controls will warn you if more than one Desired State Topic is being published to at any given time to prevent such issues. Also [TODO], if Controls stops receiving Desired State messages at a high enough rate [TBD], it will output zero power for safety purposes.

The topic that is published to can be configured in controls.launch by passing in a parameter through the command line or a parent launch file.

**Not yet, but soon**: When publishing local or global twists for desired state, every axis that is set to 0 will be controlled for by position PID rather than velocity PID to mitigate drift effects or other unwanted perturbations. Only those axes that have nonzero desires will be controlled for by velocity.


