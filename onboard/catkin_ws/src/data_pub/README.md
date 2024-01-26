# Data pub

The `data_pub` package contains scripts that interface with various sensors, currently interfacing with the IMU, Pressure Sensor (depth) and DVL. You can launch all nodes in this package using the `pub_all.launch` file.
## IMU Documentation

Publishes an `sensor_msgs/IMU` message to the `sensors/imu/imu` topic that contains information about
- orientation (using quaternions)
- angular velocity
- linear acceleration


and publishes a `sensor_msgs/MagneticField` message with magnetometer measurements to the `sensors/imu/mag` topic.

Locates the serial name of the IMU and reads its input as a string:

```
$VNQMR,-0.017057,-0.000767,+0.056534,+0.998255,+1.0670,-0.2568,+3.0696,
-00.019,+00.320,-09.802,-0.002801,-0.001186,-0.001582*65
```

This is parsed into its individual components to be published as parts of the `IMU` and `MagneticField` messages.

### Execution

First you want to make sure that both the IMU and (ADAPTER) are plugged into the robot *before* booting it up.

In this case we want to run the `data_pub` package's file `IMU.py`:
```
rosrun data_pub IMU.py
```
You can also use the launch file provided, `pub_imu.launch`.

You should see data being printed out in the following format for the `sensor_msgs/IMU` topic in this new terminal:

```
header:
 seq: ####
 stamp:
   secs:####
   nsecs: #########
  frame_id: "STRING"
 orientation:
  x: X.X
  y: X.X
  z: X.X
  w: X.X
  orientation_covariance: [-1.0, 0, 0, 0, 0, 0, 0, 0, 0]
 angular_velocity:
  x: X.X
  y: X.X
  z: X.X
  w: X.X
 angular_velocity_covariance: [-1.0, 0, 0, 0, 0, 0, 0, 0, 0]
 linear_acceleration:
  x: X.XXXXXXXXXX
  y: X.XXXXXXXXXX
  z: X.XXXXXXXXXX
 linear_acceleration_covariance: [-1.0, 0, 0, 0, 0, 0, 0, 0, 0]
 ```
## DVL Documentation
The `dvl_raw` script publishes the raw DVL data using the `dvl_raw` message from our custom messages. It publishes to topic `sensors/dvl/raw`.

The `dvl_to_odom` script converts the raw dvl data to an `odometry` message for use in other scripts. It publishes to topic `sensors/dvl/odom`.

You can launch both scripts using the `pub_dvl.launch` file.

## External Sensors Documentation

The Blue Robotics pressure sensor sends raw serial data through an arduino to the main computer. This data is filtered and then published to the `/sensors/depth` topic. It converts the data into an PoseWithCovarianceStamped message for use in sensor fusion. The same node also gets voltage data from the voltage sensor on the same arduino and is published as a Float64 to `/sensors/voltage`. The data comes on the same serial stream with tags (`P:` and `V:`) identifying pressure from voltage.

Two filters are applied to the depth:
1. Values with absolute value greater than 7 are ignored.
2. A median filter is applied to the last 3 values.

These filters are applied to eliminate noise in the data that would otherwise result in an inaccurate Z position in state.

All data in this PoseWithCovarianceStamped message is set to 0 except for the `pose.pose.position.z` value, which is set to the depth in meters. The `pose.pose.orientation` is set to the identity quaternion. Except for the `pose.pose.position.z` value, all other values are unused in sensor fusion.

The voltage is published raw.