# Python Allied Vision Package

This package contains driver code for Allied Vision GigE cameras. It uses the [pymba](https://github.com/morefigs/pymba) library for Python bindings of the Vimba library that is written in C.

## Nodes

### Single Camera
Just running the `mono_camera.py` node will print out all the available camera IDs and will attach the node to the first of those cameras. Alternatively, you can attach to a specific camera using:
```bash
ros2 run avt_camera mono_camera.py --ros-args -p camera:=<cam_name> -p camera_id:=<cam_id>
```
This will publish the camera stream to topic `/camera/<cam_name>/image_raw` and the camera info will be published to `/camera/<cam_name>/camera_info`.

The parameter `<cam_name>` is a string that represents the desired namespace of the camera and `<cam_id>` is a string that represents the unique id of the camera.


To run the right camera, you could run:
```bash
ros2 run avt_camera mono_camera --ros-args -p camera:="right" -p camera_id:="DEV_000F315C1ED5"
```
This will publish the camera stream to topic `/camera/right/image_raw` and the camera info will be published to `/camera/right/camera_info`.

To run the left camera, you could run:
```bash
ros2 run avt_camera mono_camera --ros-args -p camera:="left" -p camera_id:="DEV_000F315C1ED8"
```
This will publish the camera stream to topic `/camera/left/image_raw` and the camera info will be published to `/camera/left/camera_info`.

### Stereo Camera
You can use the `stereo_camera.py` node to run stereo cameras, or anytime you need multiple cameras. Since the parameters can be quite long, we recommend
using a launch file, such as the one provided, `stereo_camera.launch`. You can run the launch file using:
```bash
ros2 launch avt_camera stereo_camera.launch.py
```
which will start stereo camera on the left and right cameras and will publish to `/camera/left/image_raw`, `/camera/right/image_raw` and the corresponding camera info topics.

This node requires two parameters for input, `<camera>` and `<camera_id>`. `<camera>` is a list of the names of the cameras that you wish to run. `<camera_id>` is a map with keys as the names of the cameras that are defined in the list `<camera>`, and with values being the corresponding IDs of the cameras.

In the same manner as the `mono_camera.py` node, the topics published will be `/camera/<cam_name>/image_raw` where `<cam_name>` is an element in `<camera>`, the names of the cameras. The camera info will also be published in a similar manner, at topic `/camera/<cam_name>/camera_info`. There will be as many topics published as there are elements in the `<camera>` list.

## Structure
This package is composed of the following code.
* `camera.py` contains the `Camera` class, which creates a high level API for accessing camera data and wraps around the pymba library
* `mono_camera.py` is a script that contains the ROS node to drive a single camera
* `stereo_camera.py` is a script that contains the ROS node to asynchronously drive multiple cameras
