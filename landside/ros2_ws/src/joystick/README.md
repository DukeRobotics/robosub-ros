## Joystick Documentation

The joystick package publishes a `geometry_msg/Twist` message to the `controls/desired_power` topic. Currently, the supported joysticks are the Logitech Gamepad F310 and the Thrustmaster, but other Linux joysticks may be supported.

The buttons `B`, `Y`, change the joystick output.
 
 * `B` sets the joysticks to publish **lateral movement** (`X, Y`), (`Z, Yaw`) to the current topic
 
 * `Y` sets the joysticks to publish **rotation** (`Pitch, Roll`), (`Z, Yaw`) to the current topic
 
Where the joysticks input follows the form:
 
Left Stick (`Up/Down, Left/Right`), Right Stick (`Up/Down, Left/Right`)

### Setup

#### Nodes
Before launching the nodes, ensure that the joystick is plugged in and the input is being mapped to `/dev/input/js0`. 

Start the raw joystick node on the **landside** computer by executing

`
ros2 launch joystick joystick_raw.launch.py
`

Start the joystick pulishing node on either computer by executing 

`
ros2 launch joystick pub_joy.launch.py
`

### Dependencies

This parser depends on the ROS joystick driver package [`joy`](http://wiki.ros.org/joy). It requires the host OS to use the Linux kernel, regardless of whether or not you are using Docker.
