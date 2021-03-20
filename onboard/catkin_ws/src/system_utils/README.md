# System utils

This package provides various ROS interfaces to assist with evaluating system performance and also provides various system utilities. 

## System Publisher
The system publisher publishes a message of type `custom_msgs/SystemUsage.msg`. It contains data corresponding to the CPU usage (percentage and speed), the GPU usage (percentage, speed, usage, and memory), RAM usage (percentage, total used, total available), and disk usage (percentage, total used, total available). All memory items and speeds and in GB and GHz respectively. See the message declaration in the custom_msgs package for more information.

To launch the system publisher, use the command
```bash
roslaunch system_utils system_pub.launch
```
