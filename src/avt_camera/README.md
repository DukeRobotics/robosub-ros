# Python Allied Vision Node

Just running the node will print out all the available camera IDs and will attach the node to the first of those cameras. Alternatively, attach to a specific camera, see below.

To run the right camera, you could run:
```
rosrun avt_camera mono_camera.py __name:=right /mono_camera/raw:=/right/raw _camera_id:="DEV_000F315C1ED5"
```

To run the left camera, you could run:
```
rosrun avt_camera mono_camera.py __name:=left /mono_camera/raw:=/left/raw _camera_id:="DEV_000F315C1ED8"
```