# System utils

This package provides various ROS interfaces to assist with evaluating system performance and also provides various system utilities. 

## System Publisher
The system publisher publishes a message of type `custom_msgs/SystemUsage.msg`. It contains data corresponding to the CPU usage (percentage and speed), the GPU usage (percentage, speed, usage, and memory), RAM usage (percentage, total used, total available), and disk usage (percentage, total used, total available). All memory items and speeds and in GB and GHz respectively. See the message declaration in the custom_msgs package for more information.

To launch the system publisher, use the command
```bash
ros2 run system_utils system_info
```

## Remote Launch
The remote launch node allows for nodes to launch other nodes in this workspace. It has two service servers, `/start_node` and `/stop_node` of type `custom_msgs/StartLaunch.srv` and `custom_msgs/StopLaunch.srv` respectively. To start the servers, run
```bash
ros2 run system_utils remote_launch
```

To start a node with the `/start_node` service, provide the package, file, any args (leave empty if none), and if it is a launch file or not (are we running roslaunch or rosrun). It will return a pid in the response. To stop a node, simply provide this pid to the `/stop_node` process, and the node will then be terminated.

### Usage examples

#### Launch File
```bash
ros2 service call /start_node custom_msgs/srv/StartLaunch "{package: <package>, file: <launch_file>, is_launch_file: true}"
ros2 service call /stop_node custom_msgs/srv/StopLaunch "{pid: <pid>}"
```

#### Node

```bash
ros2 service call /start_node custom_msgs/srv/StartLaunch "{package: <package>, file: <node_name>}"
```

Currently, nodes cannot be stopped due to complications with `ros2 run`. This feature will be added in the future.

#### With arguments
```bash
ros2 service call /start_node custom_msgs/srv/StartLaunch "{package: <package>, file: <launch_file>, is_launch_file: true, args=[<arg1_name>:=<arg1_value>, <arg1_name>:=<arg1_value>]}"
````

