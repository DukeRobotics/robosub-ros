# Thruster Speeds Panel
Thruster Speeds Panel allows you to subscribe to the robot's `offboard/thruster_speeds` topic and displays the current
thruster speeds on the robot, as well as publish desired thruster speeds values to this topic.

## Usage
Users can switch between two modes:
- In the `SUBSCRIBING` mode, a table displays all current thruster speeds on the robot, 
as an integer value between -128 and 127. This mode is read-only, meaning you cannot modify any thruster speeds values.
- In the `PUBLISHING` mode, you can enter your desired thruster speeds values for each thruster in the table and 
publish these values to the topic `offboard/thruster_speeds`. Each input must either be an integer between -128 and 127, 
or left blank. If left blank, the input value for that thruster will be the value of the current actual speed 
as subscribed from `offboard/thruster_speeds` for that thruster. 
Click `START_PUBLISHING` to start publishing the values, and `STOP PUBLISHING` to stop publishing. Note that while 
the panel is publishing, if you want to start publishing new values, you have to click `STOP_PUBLISHING` and then click
`START_PUBLISHING` again.

## Configuration
The following constants can be modified:
- `ROBOT`: either `OOGWAY` or `CTHULHU` (default to `OOGWAY`). Determines which robot's thruster configurations 
will be considered when creating a `custom_msgs/ThrusterSpeeds` from the user's inputs.