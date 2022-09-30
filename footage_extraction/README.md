# CV Footage Extraction

## Purpose
These files are used for extracting footage / images from rosbag files (file with `.bag` as an extension).

## Usage
- See the example in the main function at the bottom of `extract_footage.py`

### Example
If the rosbag file is located at FOOTAGE_PATH, you can view info about the rosbag file by
calling `view_rosbag_info(FOOTAGE_PATH)`

```
FOOTAGE_PATH = '/root/dev/robosub-ros/footage_extraction/footage/test.bag'

footage_extractor = ROSBagFootageExtractor() 
footage_extractor.view_rosbag_info(FOOTAGE_PATH)
```

This will provide a summary of the file contents. An example of the output can be seen below.
This example output was taken from [here](http://wiki.ros.org/rosbag/Commandline#info).

```
$ rosbag info foo.bag
path:        foo.bag
version:     2.0
duration:    1.2s
start:       Jun 17 2010 14:24:58.83 (1276809898.83)
end:         Jun 17 2010 14:25:00.01 (1276809900.01)
size:        14.2 KB
messages:    119
compression: none [1/1 chunks]
types:       geometry_msgs/Point [4a842b65f413084dc2b10fb484ea7f17]
topics:      /points   119 msgs @ 100.0 Hz : geometry_msgs/Point
```

To extract the footage, take note of the topic name after viewing the rosbag info and then
call `extract_frames(FOOTAGE_PATH, TOPIC_NAME, OUTPUT_DIR)`

```
OUTPUT_DIR = '/root/dev/robosub-ros/footage_extraction/footage/'
TOPIC_NAME = "/camera/front/stream_raw"
footage_extractor.extract_frames(FOOTAGE_PATH, TOPIC_NAME, OUTPUT_DIR)
```

## File Structure
`extract_footage.py`
- This file extracts footage from a local file
- Uses function from `image_tools.py` to convert ROS Image to OpenCV format before saving

`image_tools.py`
- Contains functions to convert from ROS Image messages to numpy.ndarray / OpenCV images.
- This is an external [file found from a git repo here](https://gist.github.com/awesomebytes/958a5ef9e63821a28dc05775840c34d9)
