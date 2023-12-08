# PID Panel
PID Panel subscribes to the `/controls/pid_gains` and displays the PID gains for Velocity as well as Position, as well set desired PIDs using the service `/controls/set_pid_gains`.

## Usage
Users can switch between two tabs: `Position` and `Velocity` that change which gains are being displayed.
Every text box in the table corresponds to a PID gain that is classified by the row (axis) and column (gain type) and displays the current value.
If a text box is clicked on, the value stops updating and the the box becomes highlighted in red,
indicating that it has been edited, and users may enter a new value.
Users may set several values before pressing `Submit` which updates all of the edited PID gains to be the new values.
If users would like to cancel their changes, they can press `Reset` to revert all edited textboxes to being unedited.

## Configuration
The following constants can be modified:
- `PID_TOPIC`: `/controls/pid_gains`
- `SET_PID_SERVICE`: `/controls/set_pid_gains`