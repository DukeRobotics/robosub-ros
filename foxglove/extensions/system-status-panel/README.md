# System Status Panel
This is a panel that displays the system usage of the onboard computer.

## Usage
A table displays the current usage of CPU and RAM as published to the `/system/usage` topic.
If usage is greater than 90%, the corresponding row will have a red warning background.

## Configuration
The following constants can be modified:
- `SYSTEM_USAGE_TOPIC`: The topic that contains the system usage of the onboard computer.
This topic must comform to the `custom_msgs/SystemUsage.msg` message.