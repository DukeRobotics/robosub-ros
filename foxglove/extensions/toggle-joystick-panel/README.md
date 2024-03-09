# Toggle Joystick Panel
This is a panel to toggle joystick control on/off, as well as to publish transformed joystick inputs as a desired power.

Joystick inputs consist of movement in the x, y, z, roll, pitch, and yaw axes as well as buttons to activate/fire torpedos.

When joystick control is toggled on, a request is sent to `/controls/set_control_types` toggling each control to `DESIRED_POWER` mode. Joystick inputs are now read and interpreted as power in each direction.

When joystick control is toggled off, a request is sent to `/controls/set_control_types` toggling each control to `DESIRED_POSITION` mode. Joystick inputs are now ignored.

When joystick control is enabled, joystick inputs are published to `/controls/desired_power`.

## Usage
Connect a [Thrustmaster T.Flight HOTAS X](https://www.thrustmaster.com/products/t-flight-hotas-x/) joystick to the computer running Foxglove.

Ensure that controls is running:
```bash
roslaunch controls controls.launch
```

Click the button to toggle joystick control. If the call fails, an alert is displayed.

## Configuration
The following constants can be modified:
- `DEBUG`: If true, the transformed joystick inputs are displayed in the panel.
- `PUBLISH_RATE`: The rate (in Hz) to publish desired power.
- `SET_CONTROL_TYPES_SERVICE`: The service that handles enabling/disabling joystick control.
- `DESIRED_POWER_TOPIC`: The topic that transformed joystick inputs are published to.
- `AXIS_MAP`: The map from each direction of motion to indices in the joystick axis inputs.
- `BUTTON_MAP`: The map from each desired action to indices in the joystick button inputs.