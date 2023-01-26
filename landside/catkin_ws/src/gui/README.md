# GUI

This package provides a GUI to interact and interface with the robot. Functionality is described below.

## Running

### Task Planning
To run the task planning GUI, run
```bash
roslaunch gui gui.launch
```

This will run `rqt` with the perspective file `robosub_ros_gui.perspective` located in the config folder. If new plugins need to be added, then make sure to update the perspective file to load the right plugins.

If part of a plugin is greyed out, ensure that the appropriate service is running, as listed in the dependencies in this README under each feature.

### CV
To run the CV GUI, run
```bash
roslaunch gui cv_gui.launch
```

This will run `rqt` with the perspective file `robosub_ros_cv_gui.perspective` located in the config folder. If new plugins need to be added, then make sure to update the perspective file to load the right plugins.

If part of a plugin is greyed out, ensure that the appropriate service is running, as listed in the dependencies in this README under each feature.

## Plugins

### Controls Plugin

#### Dependencies
- `controls.launch` in controls

#### Description
The `Enable Controls` button will enable controls. To disable, click on the button again (it will have changed to disable controls).

The desired control value will display values published to `/controls/desired_pose` or `/controls/desired_twist` or `/controls/desired_power` and will display what is being published.

When no values are being published to the above topics, you may click on the `Set Desired Pose/Twist` button and it will open a dialog to set a new desired control value, and once set it will publish the value. To stop, click on the same button again.

The PID constants table will display the current PID constants. To switch between the position and velocity PID constants, select the tab you would like (labeled Position and Velocity). 

To update the PID constants, click on the `Update PID constants` button, and a dialog will open where you can set the PID constants. It can be set to the third decimal.

### Sensor Plugin

#### Dependencies
- `fuse.launch` in sensor_fusion

#### Description
The hardware status box will display the connection status of various pieces of hardware. This is determined by determining whether there are messages being published to their topics.

The `Enable Keyboard Control` button will enable keyboard control allowing you to use the keyboard to control the robot by using your keyboard to emulate a joystick. WASD controls X and Y on the vertical and horizontal respectively. IJKL controls Z and Yaw on the vertical and horizontal respectively. Arrow Keys controls Pitch and Roll on the vertical and horizontal respectively. To stop keyboard input, click the button again to disable.

The State box will display the current published state.

The `Set Current` button will open a dialog that will allow you to set the current state in the EKF filter. See the robot_localization package documentation for more information.

### Offboard Plugin

#### Dependencies
- `remote_launch.py` in system_utils
- `servo_wrapper.py` in offboard_comms
- `controls.launch` in controls

#### Description
The Thruster Output box will display the 8-value array of thruster values that is published by controls.

The `Open Thruster Tester` button will open a dialog where you may test the thrusters by using the sliders and/or number box to set the output thruster value to test if they work.

The Servo Control box contains buttons to actuate each servo (setting it to angle 180). To deactivate the servo (setting it to angle 0) then press the button again.

The `Upload Arduino Code` will upload arduino code to the arduino when pressed.

### Launch Plugin

#### Dependencies
- `remote_launch.py` in system_utils

#### Description

To launch any node on onboard, select the node from the Node Name drop down list in the Launch Plugin window. If a launch file with arguments is selected, additional textboxes will appear, with the default value (if any) prepopulated. Fill in all arguments, then click OK to launch.

The table will show you what nodes are running.

To stop a node that was run using the Launch Dialog, you may click on the item in the table and, on your keyboard, press Delete for Windows or Fn + Delete for Mac, which will stop the node.

#### Launch Argument Validation & Help

The Launch Plugin also has the capability to validate arguments input by the user before the node is launched and provide descriptions of arguments. 

To use this capability, add the `doc` attribute to `<arg>` tag in your launch file. Inside this `doc` attribute, put a JSON object.

Example:
```xml
<arg name="count" default="" doc='{"type":"int","allowEmpty"=false,"help":"Help for count"}'>
```
**There must NOT be anything other than the JSON object in the `doc` attribute for the validation to work.**

If there is an error in parsing the JSON, the user will be presented with an unrestricted textbox.

The following attributes are supported in the JSON object. Any other attributes will be ignored.

- `type`
    
    Possible Values: `int`, `double`, `bool`, `str`
    
    If `type` is `int` or `double`, the user will be presented with a textbox in which they can enter only integers or floating point values, respectively.

    If `type` is `bool`, the user will be presented with a checkbox.

    If `type` is `str` the user will be presented with a textbox in which any value can be entered (no restrictions).

    If `type` is any one of the possible values, the argument name will have a `(?)` symbol next to it and the value of `type` will be displayed in a tool tip if the user hovers over the argument name. 

    If `type` is not one of the possible values, or the `type` attribute is not provided, the user will be presented with an unrestricted textbox.

    **NOTE: The `type` attribute is only applied if the `regex` and `options` attributes are not specified.**

- `regex`

    Possible Values: Any valid regex

    If a valid `regex` is specified, the user will be presented with a textbox in which only values matching the regex can be entered. Additionally, the argument name will have the `(?)` symbol next to it and if the user hovers over the argument name, the value of `regex` will be displayed.

    If `regex` is specified but is not valid, the regex will be ignored and an unrestricted textbox will be presented to the user.

    See [this link from the PyQt documentation](https://www.riverbankcomputing.com/static/Docs/PyQt5/api/qtcore/qregularexpression.html) for more information on what is a valid regex.

    **NOTE: Any backslashes `\` in the regex must be escaped by adding another backslash in front of it so it can be read by the JSON parser.**

    **NOTE: If any value for `regex` is specified, `type` is ignored. If any value for `options` is specified, `regex` is ignored.**

- `options`

    Possible Values: Any non-empty list of strings

    If a non-empty list of strings is specified for `options`, the user will be presented with a dropdown in which only values present in the list will be available for the user to select. No value other than the ones in the `options` list can be provided to the argument from this plugin.

    If an empty list, list containing one or more non-string values, or something other than a list is provided to `options`, the user will be presented with an unrestricted textbox.

    **NOTE: If any value for `options` is specified, `regex` and `type` are ignored.**

- `help`

    Possible Values: Any non-empty string

    If a non-empty string is specified for `help`, the argument name will have a `(?)` symbol displayed next to it. If the user hovers over the argument name, a tool tip will be displayed containing the text in `help`.

    If the `help` string contains only whitespace characters (considered empty), if `help` is not a string, or if `help` is not specified at all, no help text will be displayed in the tool tip.

- `allowEmpty`

    Possible Values: `true` or `false` (case sensitive)

    If `allowEmpty` is `false`, the plugin does not allow the node to be launched with the argument empty. Empty means no value was specified for the argument or the value only contained whitespace. An error message will be shown if `allowEmpty` is `false` and the argument is empty.

    If `allowEmpty` is `true`, an empty value was specified for the argument, and the user clicks the button to launch the node, then the plugin's behavior depends on the argument's default value.
    - If no default value is specified, the plugin shows an error message and requires the user to specify a value for the argument. This is because ROS does not allow nodes to be launched with no value specified for arguments without a default value.
    - If a default value is specified but is an empty string (whitespace-only strings are also considered empty), then the node is launched as normal, as ROS allows such arguments to be launched with empty values.
    - If a default value is specified and is NOT an empty string, then a warning message is displayed asking the user to confirm they want to proceed with the launch as the value of the argument will not be empty but will rather be the default value. This is beause ROS uses the argument's default value when the argument's value is left empty in the `roslaunch` command.
    
    If `allowEmpty` is not one of the possible values, or is not specified at all, the plugin behaves as if `allowEmpty` is set to `true`.

    **NOTE: The only scenario in which an argument's value is truly empty when the node is launched is when the user has left the argument empty, `allowEmpty` is `true`, _AND_ the argument has an empty default value.**
    

### Rosbag Plugin

#### Dependences
None

#### Description

To record a bag file, click the green record button. In sequence, 3 dialogs will appear with prompts to select the topics to record, set the filename and filepath, and to provide any optional arguments to the rosbag record command. Then, the bag file will begin to record and it will be added to the dropdown containing all bag files being actively recorded.

To see information about bag files being actively recorded, select one from the dropdown and it's information will be displayed in the text area.

To stop recording a bag file, select it in the dropdown. Then, click the red stop recording button and confirm in the dialog. In a few seconds, the bag file will stop recording.

When the plugin is closed, all actively recorded bag files are stopped.

### System Usage Plugin

#### Dependencies
- `system_info_publisher.py` in system_utils

#### Description

The System Usage box displays percent CPU usage and amount of RAM used in GB.
