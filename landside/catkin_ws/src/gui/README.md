# System utils

This package provides a GUI to interact and interface with the robot. Functionality is described below

## Running

To run the GUI, run
```bash
roslaunch gui gui.launch
```

This will run `rqt` with the perspective file `robosub_ros_gui.perspective` located in the config folder. If new plugins need to be added, then make sure to update the perspective file to load the right plugins.

If part of a plugin is greyed out, ensure that the appropriate service is running, as listed in the dependencies in this README under each feature.

### Systems Plugin

#### Dependencies
- `remote_launch.py` in system_utils
- `system_info_publisher.py` in system_utils 

#### Description
To launch any node in the onboard container, you may use the Node Launch Dialog, which opens a new dialog into which you may enter your info. 

You may also use the 3 buttons and sim checkbox instead if you want to launch a commonly used file from the execute package (such as motion.launch). You may only have one file from the execute package running, so the other buttons will be disabled while a launch file is being run.

The table will show you what nodes are running.

To stop a node that was run using the Node Launch Dialog, you may click on the item in the table and press Delete (on your keyboard), which will stop the node.

To stop a node run using the buttons, you may also click on the stop button that replaced the same button to start. This will stop that specific node. 

Data is published in percentage for CPU and GB for RAM in the System Usage box.

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

####Dependencies
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

