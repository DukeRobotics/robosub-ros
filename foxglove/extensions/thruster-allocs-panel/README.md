# Thruster Alloc Panel
Thruster Allocs Panel allows you to subscribe to the robot's `controls/thruster_allocs` topic and displays the current
thruster allocs on the robot, as well as publish desired thruster allocs values to this topic.

## Usage
Users can switch between two modes:
- In the `SUBSCRIBING` mode, a table displays all current thruster allocs on the robot, 
as a float between -1 and 1, inclusive. This mode is read-only, meaning you cannot modify any thruster allocs values.
- In the `PUBLISHING` mode, you can enter your desired thruster allocs values for each thruster in the table and 
publish these values to the topic `controls/thruster_allocs`. Each input must either a float between -1 and 1, 
or left blank. If left blank, the input value for that thruster will be the value of the current actual alloc 
as subscribed from `controls/thruster_allocs` for that thruster. 
Click `START_PUBLISHING` to start publishing the values, and `STOP PUBLISHING` to stop publishing. Note that while 
the panel is publishing, if you want to start publishing new values, you have to click `STOP PUBLISHING` and then click
`START PUBLISHING` again.

## Configuration
The following constants can be modified:
- `ROBOT`: either `OOGWAY` or `CTHULHU` (default to `OOGWAY`). Determines which robot's thruster configurations 
will be considered when creating a `controls/thruster_allocs` from the user's inputs.