## Joystick Documentation

The joystick publishes a `geometry_msg/Twist` message to the `controls/desired_twist_power` topics.

The buttons `X`, `Y`, change the joystick output.
 
 * `X` sets the joysticks to publish **lateral movement** (`X, Y`), (`Z, Yaw`) to the current topic
 
 * `Y` sets the joysticks to publish **rotation** (`Pitch, Roll`), (`Z, Yaw`) to the current topic
 
Where the joysticks input follows the form:
 
Left Stick (`Up/Down, Left/Right`), Right Stick (`Up/Down, Left/Right`)
 
### Dependencies

This parser depends on the ROS joystick driver package `joy`.

