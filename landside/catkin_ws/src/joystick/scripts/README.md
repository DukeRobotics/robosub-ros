## Joystick Documentation

The joystick package publishes a `geometry_msg/Twist` message to the `controls/desired_twist_power` topics. Currently, the supported joysticks are the Logitech Gamepad F310 and the Thrustmaster, but other Linux joysticks may be supported.

The buttons `B`, `Y`, change the joystick output.
 
 * `B` sets the joysticks to publish **lateral movement** (`X, Y`), (`Z, Yaw`) to the current topic
 
 * `Y` sets the joysticks to publish **rotation** (`Pitch, Roll`), (`Z, Yaw`) to the current topic
 
Where the joysticks input follows the form:
 
Left Stick (`Up/Down, Left/Right`), Right Stick (`Up/Down, Left/Right`)

### Setup

#### Network
First, run roscore on the **onboard** computer. Then set up the network on the **landside** computer by executing

`
export /opt/ros/melodic/pool_setup.bash
`

on the landside computer, if you are using the landside image. Otherwise you may execute

`
export ROS_MASTER_URI=http://192.168.1.1:11311
`

#### Nodes
Before launching the nodes, ensure that the joystick is plugged in and the input is being mapped to `/dev/input/js0`. 

Start the raw joystick node on the **landside** computer by executing

`
roslaunch joystick_raw.launch
`

Start the joystick pulishing node on either computer by executing 

`
roslaunch pub_joy.launch
`

### Dependencies

This parser depends on the ROS joystick driver package `joy`. It requires the Linux kernel.
