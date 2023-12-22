# PID Panel
This is a panel to read/set PID gains.

## Usage
Use the tabs to switch between the two PID loop types (Position/Velocity).
Each field in the table corresponds to a PID gain that is classified by the row (axis) and column (gain type).

By default, each field displays its current PID value. If a field is clicked on, the value stops updating and the field becomes highlighted in red, indicating that it has been edited, and users may enter a new desired value.
Users may propose new values for multiple gains before clicking "Submit" which then updates all of the edited PID gains.

If users would like to cancel their changes, they can click "Reset" to revert all edited fields back to their read state.

## Configuration
The following constants can be modified:
- `PID_TOPIC`: The topic where the current PID gains are published.
- `PID_SERVICE`: The service which handles setting new PID gains.