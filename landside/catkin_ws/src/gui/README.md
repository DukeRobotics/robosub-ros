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

The table will show you what nodes are running. To view information about any node, you may double click on that row. An info dialog will show up that either displays the PID, Package Name, File Name, and Args of the node if it is still running, or that the node is already terminated.

To stop a node that was run using the Launch Dialog, you may click on the item in the table and, on your keyboard, press Delete for Windows or Fn + Delete for Mac, which will stop the node.

To open the configuration dialog, you may click on the gear icon in the top right corner of the plugin. In the configuration dialog, there are 3 independent checkboxes:

- Display all nodes: if checked, the plugin will display all nodes launched by remote_launch (not just nodes launched by the plugin) and allow the user to terminate any node that is displayed in the table. This means that if there is more than one instance of the Launch Plugin being used to launch nodes, then all instances will display all nodes that have been launched by all instances and they all can terminate each others' node as well.
- Only remove nodes manually: if checked, the plugin does not remove nodes that were terminated on their own or by another launch plugin from the table; nodes will only be removed from the table when the user selects a row and presses Delete/Fn + Delete. If the row is a running node, it will be terminated before being removed from the table. Otherwise, if this option is not checked, whenever a node is terminated (either on its own by Delete/Fn + Delete), it will automatically be removed from the table.
- Display running status: if checked, the plugin adds a column "Status" to the table that indicates whether each node is currently running or has been terminated.


#### Launch Argument Validation & Help

The Launch Plugin also has the capability to validate arguments input by the user before the node is launched and provide descriptions of arguments. 

To use this capability, add the `doc` attribute to `<arg>` tag in your launch file. Inside this `doc` attribute, put a JSON object.

Example:
```xml
<arg name="count" default="" doc='{"type":"int","allowEmpty"=false,"help":"The number of messages to publish"}' />
```
**There must NOT be anything other than the JSON object in the `doc` attribute for the validation to work.**

If there is an error in parsing the JSON, the user will be presented with an unrestricted text input.

The following table lists all attributes and values that can be part of the JSON object that are interpreted by this plugin. For each combination of attribute and possible value, the "Impact" column describes the change in the plugin's UI or behavior that will result from the combination being part of the JSON object. This "Impact" will materialize _if and only if_ the argument's default value is either not set or is part of the "Accepted Defaults" for the combination of attribute and possible value. For combinations that have a listed tooltip, the tooltip will be displayed when the user hovers over the argument name.

<table>
<thead>
  <tr>
    <th>Attribute</th>
    <th>Possible Values</th>
    <th>Impact</th>
    <th>Accepted Defaults</th>
    <th>Tooltip</th>
  </tr>
</thead>
<tbody>
  <tr>
    <td rowspan="4">type</td>
    <td>"int"</td>
    <td>Text input restricted to integers only</td>
    <td>Any integer or empty string</td>
    <td>"Type: int"</td>
  </tr>
  <tr>
    <td>"double"</td>
    <td>Text input restricted to floating point values only</td>
    <td>Any floating point value or empty string</td>
    <td>"Type: double"</td>
  </tr>
  <tr>
    <td>"bool"</td>
    <td>Checkbox</td>
    <td>"true" or "false" (case-insensitive)</td>
    <td>"Type: bool"</td>
  </tr>
  <tr>
    <td>"str"</td>
    <td>Text input unrestricted</td>
    <td>Any string</td>
    <td>"Type: str"</td>
  </tr>
  <tr>
    <td>regex</td>
    <td>Any valid regex in a string*</td>
    <td>Text input restricted to match regex</td>
    <td>Any string matching the regex or empty string</td>
    <td>"Regex: [provided_regex]"</td>
  </tr>
  <tr>
    <td>options</td>
    <td>Any non-empty list of strings</td>
    <td>Dropdown only containing values from list</td>
    <td>Any value in the list**</td>
    <td>None</td>
  </tr>
  <tr>
    <td>help</td>
    <td>Any non-empty string</td>
    <td>See Tooltip</td>
    <td>N/A</td>
    <td>"Help: [provided_help_text]"</td>
  </tr>
  <tr>
    <td rowspan="2">allowEmpty</td>
    <td>true (no quotes, case-sensitive)</td>
    <td>Plugin allows argument value to be empty string†</td>
    <td>Empty string†</td>
    <td>None</td>
  </tr>
  <tr>
    <td>false (no quotes, case-sensitive)</td>
    <td>Plugin requires argument value to be a empty string</td>
    <td>Any string</td>
    <td>None</td>
  </tr>
</tbody>
</table>

\* See [this page in the PyQt documentation](https://www.riverbankcomputing.com/static/Docs/PyQt5/api/qtcore/qregularexpression.html) for more information on what is a valid regex. The regex must be in a string. Additionally, any backslashes `\` in the regex must be escaped by adding another backslash in front of it so it can be read by the JSON parser. For example, if you want to validate the input is an IPv4 address uisng the regex
```re
^((25[0-5]|(2[0-4]|1\d|[1-9]|)\d)\.?\b){4}$
```
the regex attribute in the JSON object would be

```json
doc='{"regex":"^((25[0-5]|(2[0-4]|1\\d|[1-9]|)\\d)\\.?\\b){4}$"}'
```

\** If the default value is not in the list, it will be added as the first item of the list.

† If `allowEmpty` is set to `true`, then the plugin does _not_ automatically allow the node to be launched with the empty argument. Instead, the plugin's behavior depends on the argument's default value
- If no default value is specified, the plugin shows an error message and requires the user to specify a value for the argument. This is because ROS does not allow nodes to be launched with no value specified for arguments without a default value.
- If a default value is specified but is an empty string (whitespace-only strings are also considered empty), then the node is launched as normal, as ROS allows such arguments to be launched with empty values.
- If a default value is specified and is NOT an empty string, then a warning message is displayed asking the user to confirm they want to proceed with the launch as the value of the argument will not be empty but will rather be the default value. This is beause ROS uses the argument's default value when the argument's value is left empty in the `roslaunch` command.
- TLDR: The only scenario in which an argument's value is truly empty when the node is launched is if `allowEmpty` is `true`, the user has left the argument empty, _AND_ the argument has an empty default value.**

A few more notes about the table:
- An "empty string" is a string that either:
    - Contains no characters OR
    - Contains only whitespace
- For a given attribute, its value is "valid" if the value is one of the Possible Values associated with the attribute in the table.
- An attribute is "specified" if it is set to some value, regardless of if the value is valid or if the default is part of the accepted defaults.
- Out of these three attributes – `options`, `regex`, and `type` – only one attribute should be present in the JSON object.
- If the `options` attribute is specified, the `regex` and `type` attributes are ignored. If the `options` attribute is not specified and the `regex` attribute is specified, the `type` attribute is ignored.
- If none of these attributes – `options`, `regex`, and `type` – are specified, then an unrestricted text input is displayed. 
- If `allowEmpty` is not specified, the plugin behaves as if it were set to `true`.
- If an argument has one or more tooltips, the argument name will have a `(?)` symbol displayed next to it. If the user hovers over the argument name, all tooltips will be displayed together. Ex. "Type: bool | Help: The publishing rate."


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

### Camera Status Plugin

#### Dependences
- `camera_test_connect.launch` in cv
- `ping_host.launch` in cv
- `serial.launch` in offboard_comms
- `camera_hard_reset.launch` in cv

#### Description
If `ping_host.launch` is running this plugin shows whether the ping to the stereo camera was successful.

To check whether the mono camera is connected, click the mono button.
To check whether the stereo camera is connected, click the stereo button.

To view all recorded ping, mono, and stereo connections, click the logs button. A dialog will appear with tabs along the top that allow you to view the history of ping, stereo, and mono connections.

From the perspective file, you can control the hostname whose ping messages are displayed and the channel that is checked for the mono camera.

Oogway robot: If `serial.launch` and `camera_hard_reset.launch` is running, this plugin shows the current relay status.
To restart the DepthAI camera, click the 'Turn On/Off' button. The button will reenable again once the relay successfully switches state.