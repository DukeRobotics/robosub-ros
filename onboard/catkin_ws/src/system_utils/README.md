# System Utils

This package provides various ROS interfaces to assist with evaluating system performance and also provides various system utilities.

## System Publisher
The system publisher publishes a message of type `custom_msgs/SystemUsage.msg`. It contains data corresponding to the CPU usage (percentage and speed), the GPU usage (percentage, speed, usage, and memory), RAM usage (percentage, total used, total available), and disk usage (percentage, total used, total available). All memory items and speeds and in GB and GHz respectively. See the message declaration in the custom_msgs package for more information.

To launch the system publisher, use the command
```bash
roslaunch system_utils system_pub.launch
```

## Remote Launch
The remote launch node allows for nodes to launch other nodes in this workspace. It has two service servers, `/start_node` and `/stop_node` of type `custom_msgs/StartLaunch.srv` and `custom_msgs/StopLaunch.srv` respectively. To start the servers, run
```bash
roslaunch system_utils remote_launch.launch
```

## Sensor Check
The sensor check script will enable the user to automatically check if the robot's most significant sensors are publishing. The topics which `sensor_check.py` will check are
- DVL - `/sensors/dvl/odom`
- IMU - `/vectornav/IMU`
- Depth sensor - `/sensors/depth`
- State - `/state`
- Stereo camera - `/camera/front/rgb/preview/compressed`
- Sonar - `/sonar/status`
To add more sensors to check, simpply include the name of the rostopic and the message type under `SENSOR_SUBSCRIBE_TOPICS`. To run the script, use the command
```bash
roslaunch system_utils sensor_check.launch
```

To start a node with the `/start_node` service, provide the package, file, any args (leave empty if none), and if it is a launch file or not (are we running roslaunch or rosrun). It will return a pid in the response. To stop a node, simply provide this pid to the `/stop_node` process, and the node will then be terminated.

## Topic Transforms

The topic transforms node will allow for the user to transform a topic from one message type to another. This is useful primarily for debugging purposes, or to make certain topics more human-readable. To run the node, use the command
```bash
roslaunch system_utils topic_transforms.launch
```

To transform a topic, add a `TopicTransformData` object to the `TOPIC_TRANSFORM_DATA` in the `TopicTransforms` class. 

The purpose of the `input_type_conversion` function is to convert the topic's message type to a common message type that can be input to the `output_type_conversion` function. This avoids code repetition when several input topics have different input types but have the same output type. 

For example, suppose we have two topics: one with input type `nav_msgs/Odometry` and another with input type `geometry_msgs/Pose`. The transformed output of both is of type `geometry_msgs/Twist`, representing the odometry.pose.pose and pose, respectively, converted to twists with Euler angles in place of quaternions. In this case, creating two separate functions, one to convert odometry to twist, and one to convert pose to twist would be redundant, since in both cases the actual transform being done is pose to twist. Therefore, to avoid code repetition, use `input_type_conversion` to simply return the pose component of the odometry message, then use the pose to twist function as the `output_type_conversion` function for both.

The `input_type_conversion` function should almost always be a `lambda` function, as it's purpose is to only return a specific componenent of the input message. The `output_type_conversion` function is the one that does all the heavy lifting and applies the desired transform.
