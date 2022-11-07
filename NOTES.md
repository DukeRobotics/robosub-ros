# Notes
To source ROS: `source /opt/ros/humble/setup.bash`

custom_msgs takes very long to build (around 90 seconds) with colcon. We should consider pre-building custom_msgs for the core DockerFile if possible.

## Bulding
For building, use `colcon build --symlink-install`, because colcon has no devel space. This allows us to change scripts without rebuilding for each change.

Post colcon build, run

```bash
source landside/ros2_ws/install/setup.bash
source onboard/ros2_ws/install/setup.bash
```

No `colcon clean`, instead `rm -r build install log`

## Writing nodes
Nodes now inherit from `rclpy.node.Node`, and are created via `super().__init__(self.NODE_NAME)`

`main()` should now include `rclpy.init()`

`rospy.spin()` should be moved from inside of node to `main()`

```python
try:
    rclpy.init(args=args)
    parser = JoystickParser()
    rclpy.spin(parser)
except KeyboardInterrupt:
    pass
except ExternalShutdownException:
    sys.exit(1)
finally:
    parser.destroy_node()
    if rclpy.ok():
    	rclpy.shutdown()
```

Subscribers need QoS (Quality of Service) profiles. This is the 4th positional argument, use 10 for a default value.

`self.create_subscription(SimObjectArray, "/sim/object_points", self.callback, 10)`

### Time
Replace `rospy.Time.now()` with `self.get_clock().now()`

Replace `while not rospy.is_shutdown():` with `while rclpy.ok():`

Have to convert time to messages explicitly: `.to_msg().sec`

### Run Loops
Previously, run loops were written as:

```python
def run(self):
    rate = rospy.Rate(self.RUN_LOOP_RATE)
    while not rospy.is_shutdown():
        # Do stuff
        rate.sleep()
```

In ROS2, a way to do this is to create timers:

```python
class TestNode(Node)
    def __init__(self):
        super().__init__(self.NODE_NAME)
        self.timer = self.create_timer(1/self.RUN_LOOP_RATE, self.run)

    def run(self):
        # Do stuff


def main():
    tn = TestNode()
    rclpy.spin(tn)
```

This allows us to take advantage of threading and have multiple "loops" running in a node cleanly. There are other methods, but this is generally the cleanest.

## Transforms
Replace `tf` with `tf_transformations`

Replace `import tf2` with `import tf2_ros`

To declare a transform listener in a node and transform a PoseStamped:
```python
import rclpy
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_geometry_msgs import do_transform_pose_stamped


class TestNode(rcply.node.Node):
    super().__init__("node_name")
    self.tf_buffer = Buffer()
    self.tf_listener = TransformListener(self.buffer, self)

    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = "source_frame"
    transform = self.tf_buffer.lookup_transform("target_frame",
                                                poses.header.frame_id,
                                                rclpy.time.Time())
    transformed_pose_stamped = do_transform_pose_stamped(poses, transform)
```


## ROSbags

Rosbags are no longer stored as .bag files. They are now stored as sqlite3 databases.

## Dependencies

### Core
```bash
pip install transforms3d
pip install setuptools==58.2.0
apt-get install ros-humble-resource-retriever
apt-get install ros-humble-cv-bridge
apt-get install ros-humble-tf2
apt-get install ros-humble-tf-transformations
apt-get install ros-humble-tf2-geometry-msgs
```

### Landside
```bash
apt-get install ros-humble-joy
apt-get install ros-humble-image-view
apt-get install ros-humble-image-publisher
apt-get install ros-humble-rqt
```

### Onboard
```bash
apt-get install ros-humble-robot-localization
```

## Testing Plan
Highlighting some general functionality that we need to cover in our testing plan. We will eventually write a GitHub issue for this.

### Landside

#### camera_view
1. Record bag from cameras using ROS2 bag cli. Add usage to the README
1. Run `bag_to_video` to convert the bag to an avi file, and verify that the feed looks correct
1. Run `video_to_bag` to convert the avi file back to a bag file. Note that the size should increase drastically due to frame padding, this is expected (use a small original bag file).
1. View stereo and mono camera feeds from landside while `avt_camera` is publishing. Update README to include new commands for viewing feeds.

#### joystick
1. Connect F310 joystick to the robot computer and run `F310.launch.py`. Verify that `joystick/raw` and `controls/desired_power` are receiving messages.
1. Connect Thrustmaster joystick to the robot computer and run `thrustmaster.launch.py`. Verify that `joystick/raw` and `controls/desired_power` are receiving messages.

#### simulation
1. Run `test_sim_comm.launch.py` in an empty scene and verify that there are no errors (the robot doesn't need to move in a square).
1. Run `test_sim_comm.launch,py` in a scene with an object (i.e. gate) and verify that `fake_cv_maker.py` works correctly.

### Onboard
#### acoustics
1. Run `acoustics.launch.py` in simulation and use the ROS2 action cli to generate some sample data. Verify that the results are expected.

#### avt_camera
1. Connect the left and right Allied Vision cameras to the robot computer and run `mono_camera` on both. Verify that the cameras connect and the corresponding topics are being published to (`camera/left/image_raw` and `camera/left/camera_info`)
1. Run `stereo_cameras.launch.py` and verify that all of the corresponding topics are being published.

#### data_pub
1. Run `pub_dvl.launch.py` and verify that the computer connects to the dvl. Verify that the `dvl/raw` and `dvl/odom` topics are being published to. Make sure the data is reasonable and the publishing rate is adequate.
1. Run `pub_imu.launch.py` and verify that the computer connects to the imu. Verify that the `sensors/imu/imu` and `sensors/imu/mag` topics are being published to. Make sure the data is reasonable and the publishing rate is adequate.
1. Run `pub_depth.launch.py` and verify that `offboard/pressure` is receiving values and `sensors/depth` is being published to. Make sure the data is reasonable and the publishing rate is adequate. This requires`offboard_comms` to be running to receive pressure sensor data from the Arduino.
1. Run `pub_all.launch.py` to make sure all of the sensors work together.

#### sensor_fusion
1. Run `fuse.launch.py` while the DVL and IMU are publishing. Verify that `/state` is published and has reasonable values.
1. Verify that we don't need to publish `robot_description` to get tf2 transforms. Our old documentation says that this is needed but I don't think this is the case anymore.

#### static_transforms
1. Run `static_transforms.launch.py` and verify that the correct transform values are being published.

