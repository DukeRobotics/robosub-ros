# Data pub

The `data_pub` package contains scripts that interface with various sensors, currently interfacing with the IMU and DVL. You can launch all nodes in this package using the `pub_all.launch` file.
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

In the transition from the old computer (`NUC`) to the new computer (`Jetson`), certain physical devices need to be checked before running code.

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
