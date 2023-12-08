# Toggle Joystick Panel
This is a panel to toggle joystick control on and off, as well as publish joystick inputs.

Joystick inputs consist of movement in the x, y, and z directions, as well as roll, pitch, yaw, a button to activate the torpedos, and two buttons to fire torpedos.

When joystick control is toggled on, a request is sent to `/controls/set_control_types` toggling each control to `DESIRED_POWER` mode. Joystick inputs are now read and interpreted as power in each direction, or button presses.

When joystick control is toggled off, a request is sent to `/controls/set_control_types` toggling each control to `DESIRED_POSE` mode. Joystick inputs are now ignored.

When joystick control is enabled, joystick inputs are published to `/controls/desired_power`.

## Usage
Ensure that the `/controls/set_control_types` service is running. Click the button to toggle joystick control. If the call fails, an alert is displayed.

## Configuration
The following constants can be modified:

- `SET_CONTROL_TYPES_SERVICE`: The service that handles enabling/disabling joystick control.
- `DESIRED_POWER_TOPIC`: The service that joystick inputs are published to.
- `AXIS_MAP`: The map from each direction of motion to indices in the joystick axis inputs. 
- `BUTTON_MAP`: The map from each desired action to indices in the joystick button inputs.