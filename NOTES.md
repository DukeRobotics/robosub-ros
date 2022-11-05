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

## Time
Replace `rospy.Time.now()` with `self.get_clock().now()`
Replace `while not rospy.is_shutdown():` with `while rclpy.ok():`
Have to convert time to messages explicitly: `.to_msg().sec`

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
	pip install setuptools==58.2.0
    pip install transforms3d
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
````