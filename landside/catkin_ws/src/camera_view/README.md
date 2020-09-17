# Camera view

This metapackage contains the necessary tools to view, save, and load video streams/files.

## Viewing Videos
### Single Video Stream
You can view a live video stream by using the command
```bash
rosrun image_view image_view image:=<image_topic>
```
where `image_topic` is the topic of the published video stream. For example, to view the downward camera, you would use
```bash
rosrun image_view image_view image:=/camera/down/raw_image
```
where the subscribed topic is `/camera/down/raw_image`.

### Stereo Video Stream
You can view a stream from stereo cameras using the command
```bash
rosrun image_view stereo_view stereo:=<stereo_namespace> image:=<image_topic>
```
where stereo namespace represents the overall namespace of the cameras and image topic is the specific image identifier. For example, to view our left/right stereo cams you would use
```bash
rosrun image_view stereo_view stereo:=camera image:=raw_image
```
where the subscribed topics are `/camera/left/raw_image` and `/camera/right/raw_image`.

## Saving Videos
### ROS bags for saving videos
You can save your video stream to ROS bags for easy playback to ROS topics. Be warned that this will take significant storage space. To record a basic ROS bag, use
```bash
rosbag record -O output_name <topic_names>
```
where `output_name` is the name of the output file and `topic_names` are a list of topics to record. For example, to record the downward camera, use
```bash
rosbag record -O down_cam.bag /camera/down/raw_image
```

### Video converter
You can also use the video recorder tool in ROS image viewer to record video files. By default it encodes the video as MPG, encapsulated in a AVI container at 15 fps, and produces a file called output.avi in the current directory, but this can be changed. For more info, view the documentation [here](http://wiki.ros.org/image_view#image_view.2Fdiamondback.video_recorder).

To record, use the command
```bash
rosrun image_view video_recorder image:=<image_topic>
```
where `image_topic` is the topic of the video steam.

## Streaming Videos
### ROS bags for streaming videos
To stream a ROS bag from a bag file, you can simply use
```bash
rosbag play <bag_files>
```
where `bag_files` are the file locations of the bag files.
### Video files
You can stream avi motion files using the image publisher node. To do this, use
```bash
rosrun image_publisher image_publisher image_raw:=<image_topic>
```

## Converting between ROS bags and Video files

### ROS Bag to Video file
You can use the `bag_to_video` script provided using
```bash
rosrun camera_view bag_to_video.py _bag_file:=<bag_file_name> _video_file:=<list_of_video_files> _topic_name:=<list_of_topic_names>
```
where the `bag_file_name` is the name of the bag file to convert. The `list_of_video_files` is a list of the video files to write. The `list_of_topic_names` is a list of topic names that correspond to the topic names to write to video files. Each video file will be the stream of images to the corresponding topic at the same index in the topic list. The video file list and topic list must be the same length.
For instance, you can run
```bash
rosrun camera_view bag_to_video.py _bag_file:="vid.bag" _video_file:=["vid1.avi", "vid2.avi"] _topic_name:=["/camera/image/vid1", "/camera/image/vid2]
```

### Video file to ROS Bag
You can use the `video_to_bag` script provided using
```bash
rosrun camera_view video_to_bag.py _bag_file:=<bag_file_name> _video_file:=<list_of_video_files> _topic_name:=<list_of_topic_names>
```
which has the same parameters as in the ROS bag to video file converter. Multiple video files can be written to the same ROS bag using this script.
