## Joystick Documentation

The joystick publishes a `geometry_msg/Twist` message to the `controls/desired_twist_local` or `controls/desired_twist_global` topics, depending on the publishing mode (`local/global`).

The joystick itself produces an integer in range `[-127,127]`.

This output is divided by `128` to produce a `float` in the range `[-0.9921875, 0.9921875]` to prevent edge cases.

*The `joystick.py` script can be changed to divide by `127` to produce a range of `[-1, 1]` if needed.*

The buttons `A`, `B`, `X`, `Y`, change the joystick output.

 * `A` sets the joysticks to publish data to the **`controls/desired_twist_local`** topic
 
 * `B` sets the joysticks to publish data to the **`controls/desired_twist_global`** topic
 
 * `X` sets the joysticks to publish **lateral movement** (`X, Y`), (`Z, Yaw`) to the current topic
 
 * `Y` sets the joysticks to publish **rotation** (`Pitch, Roll`), (`Z, Yaw`) to the current topic
 
Where the joysticks input follows the form:
 
Left Stick (`Up/Down, Left/Right`), Right Stick (`Up/Down, Left/Right`)
 
### Dependencies

The controller must have the toggle switch on the **back** of the controller set to `X`, for `XInput` mode.

The parser code was repurposed from https://github.com/JohnLZeller/F310_Gamepad_Parser

The code runs properly on `python2` and **not** on `python3`.

This parser has dependencies on; however, not all of these may be required due to local changes:

 * `python`
 * `python-pmw`
 * `python-imaging`
  
The parser depends on the `core/parser_core.py` file within the `f310` directory.  This core processes the binary data given by the controller to the computer.  This data is stored within the `parsercore` object when `joystick.py` is run.
