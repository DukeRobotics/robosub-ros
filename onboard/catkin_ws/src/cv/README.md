# Computer Vision

The computer vision package listens for images/frames coming from multiple cameras. The package
will then run pre-trained machine learning models frames from each camera and output bounding boxes for the various objects
in the frames. These objects could be the gate, buoys, etc. The package will publish to different topics depending
on which classes are being detected and which cameras are being used.

## DepthAI Camera
This package contains code for the Luxonis OAK-D PoE camera, which uses a python package called [depthai](https://docs.luxonis.com/en/latest/). This camera handles neural network and image processing on the camera's processor, which necessitates a different code structure. Because of this, we have DepthAI-specific scripts and launch files that can be run for any Luxonis / DepthAI camera. For running other cameras, see the instructions below, titled [Non-DepthAI Cameras](#non-depthai-cameras).

### Running the Code
To stream the feed or perform spatial detection using the OAK camera, use `roslaunch` with the following three files.
* `depthai_camera_connect.launch`: Connects to the DepthAI camera. If connection is successful, prints a success message to console. If connection is unsucessful, an error is raised.
* `depthai_publish_image_stream.launch`: Streams the live feed from the camera. You can choose what to publish from the camera (rgb video, rgb preview, left mono, right mono, disparity map, and depth map) by setting the appropriate boolean parameters.
* `depthai_spatial_detection.launch`: Runs spatial detection. Waits for a enable_model rosservice call to specify what model to activate. This requires a valid `.blob` file in `models/` and the path to this `.blob` file should be specified in the `depthai_models.yaml` file. For more information about these files, see the code structure outline below. This will publish `CVObject` messages to a topic for each class that the model detects, unaltered rgb preview frames that were input to the neural netowrk, and rgb preview frames with bounding boxes, classes, and confidence values overlaid.
* `depthai_simulate_detection.launch`: Runs spatial detection on a still image, or on a image stream launched by running `test_images.py` [(see Simulating Image Feeds)](#simulating-image-feeds), on the DepthAI camera. Uses the model specified in arguments. If a still image is input, a JPEG file will be created that is the original image with detections visualized. If an image stream is input, CVObject messages will be published to the topic provided (all classes are published to a single topic), and a live feed of images with detections visualized is also published.

### Structure
`scripts/`
* `depthai_camera_connect.py`: Connects to the OAK camera and uploads the image pipeline. Used by all other DepthAI scripts.
* `depthai_publish_image_stream.py`: Publishes a preview of the image feed from the OAK camera. This can be used to verify connection to the camera and to check if there are any issues with the camera feed.
* `depthai_spatial_detection.py`: Waits for an enable_model rosservice call, and then publishes spatial detections using the model specified in the service call and in depthai_models.yaml.
* `depthai_simulate_detection.launch`: Runs spatial detection on a user-specified DepthAI model using a still image or image feed as input.

`launch/`
* `depthai_camera_connext.launch`: Connects to the OAK camera and uploads the image pipeline.
* `depthai_publish_image_stream.launch`: Runs the image stream script.
* `depthai_spatial_detection.launch`: Runs the spatial detection script.
* `depthai_spatial_detection.launch`: Runs the simulated spatial detection script.

`models/`
* `depthai_models.yaml`: contains models for object detection. A model is specified by a name, what classes it predicts, and the path to a .blob file, as well as other configuration parameters. `input_size` is [width, height]. The blob file format is specific to the processors that the OAK cameras use.

`footage_extraction`
* Can be used to extract footage from rosbag files. See the README file in the footage_extraction directory.

# Non-DepthAI Cameras

## USB Camera
This package also contains driver code to publish a camera stream from a USB-type camera in `usb_camera.py`. A USB camera can be located by `/dev/video*` on a linux computer, where `*` can be replaced by any number specifying a given camera channel (default is `0`, with the number increasing for each new camera you plug in). The script `usb_camera.py` uses OpenCV to capture a stream frame by frame from a specified USB camera channel and publishes it to a specified ros topic. Use `roslaunch cv usb_camera.launch` to start a stream once a USB camera has been plugged in. You can specify the ros topic which the usb camera feed is published to via

```bash
roslaunch cv usb_camera.launch topic:=<topic>
```

By default, `<topic>` is set to `/camera/usb_camera/compressed`. Note that the camera must be plugged in _before_ the docker container is started.

## Setup

Generally, you would train a separate object detection model for each task you need computer vision for (gates, buoys, etc.). You can then load them as follows:

* Create object detection models and save them as .pth files (see [here](https://github.com/DukeRobotics/documentation/tree/master/cv/training))
* Place these models in the `/models` folder
* Update the `/models/models.yaml` file with each model's details in the following format:

```yaml
<model_name>:  # A name/identifier for your model
  classes: [<class1>, <class2>, ...]  # The classes the model is trained to predict
  topic: <topic_name>  # set to /cv by default; change if you want to specify model in publisher topics .etc
  weights: <file_name>  # the name of your model file
...
```

Example entry for a buoy model:

```yaml
buoy:
  classes: [alien, bat, witch]
  topic: /cv
  weights: buoy_model.pth
```

Note: To get the model files onto the docker container, you may have to use `scp`. Also, if you come across the following error:

`URLError: <urlopen error [Errno -3] Temporary failure in name resolution>`

Navigate to [this url](https://download.pytorch.org/models/fasterrcnn_resnet50_fpn_coco-258fb6c6.pth)
to manually download the default model file used by the Detecto package. Move this file onto the Docker
container under the directory `/root/.cache/torch/checkpoints/` (do not rename the file).


## Running

To start up a CV node, run the following command:

```bash
roslaunch cv cv.launch camera:=<camera>
```

where `<camera>` refers to the topic which the camera feed is published to. For usb cameras and local simulated streams (i.e., a stream launched via `test_images.py`), this refers to the `<topic>` parameter passed in when you launched those files.

After starting up a CV node, all models are initially disabled. You can select which model(s) you
want to enable for this camera by using the following service (where `<camera>` is the value you
chose above):

* `enable_model_<camera>`
  * Takes in the model name (string) and a boolean flag to specify whether to turn the model on or off
  * Returns a boolean indicating whether the attempt was successful
  * Type: custom_msgs/EnableModel
  * E.g. `rosservice call enable_model_left buoy true` would enable the buoy model on the camera launched with `<camera>` set to `"left"`

Once 1+ models are enabled for a specific node, they listen and publish to topics as described below in topics.

## Topics

#### Listening:

 * `/camera/<camera>/compressed`
   * The topic that the camera publishes each frame to
   * If no actual camera feed is available, you can simulate one using `test_images.py`. See Simulating image feeds above for more information.
   * Type: sensor_msgs/CompressedImage

#### Publishing:

* `cv/<camera>/<class_name>`
  * For each camera frame feed that a model processes, it will publish predictions to this topic
  * `<class_name>` corresponds to one specific `class` under the `models.yaml` file for the enabled model
    (e.g. the example `bat` class above will publish to `/cv/left/bat`)
  * For each detected object in a frame, the model will publish the `xmin`, `ymin`, `xmax`, and `ymax`
    coordinates (normalized to \[0, 1\], with (0, 0) being the top-left corner), `label` of the object, `score` (a confidence value in the range
    of \[0, 1\]), and the `width` and `height` of the frame.
  * If a model is enabled but detects no objects in a frame, it will not publish any messages to any topic
  * Type: custom_msgs/CVObject

Note that the camera feed frame rate will likely be greater than the rate at which predictions can
be generated (especially if more than one model is enabled at the same time), so the publishing rate
could be anywhere from like 0.2 to 10 FPS depending on computing power/the GPU/other factors.

## Structure

The following are the folders and files in the CV package:

`assets`: Folder with a dummy image to test the CV package on

`launch`: Contains the various launch files for our CV package. There is a general launch file for all the cameras (`cv.launch`), and then there are specific launch files for each camera (`cv_left`, `cv_right`, and `cv_down`). Finally, we have a launch file for our testing script `test_images.launch`

`models`: Contains our pre-trained models and a `.yaml` file that specifies the details of each model (classes predicted, topic name, and the path to the model weights)

`scripts`: This is the "meat" of our package. We have a detection script `detection.py` that will read in images and publish predictions onto a node. We also have a `test_images.py` script that is used for testing our package on a dummy video feed, generated either through looping an image or folder of images, or running a video or rosbag file. The path of this file is passed when `test_images.py` is launched. We can simulate different video feeds coming in on the different cameras on our `test_images.py` script.

`CMakeLists.txt`: A text file stating the necessary package dependencies and the files in our package.

`package.xml`: A xml file stating the basic information about the CV package

The CV package also has dependencies in the `core/catkin_ws/src/custom_msgs` folder.

## Examples
To simulate camera feed and then run a model on the feed from the left camera. We'll assume the model we want to run is called 'buoy':
* In one terminal, run `roslaunch cv test_images.launch feed_path:=../assets/{file_name} topic:={topic_name} framerate:={required framerate}` to start the script meant to simulate the raw camera feed
* In a new terminal, run `roslaunch cv cv_left.launch` to start the cv node
* In another new terminal, run `rosservice list` and should see the `enable_model_left` service be listed in the terminal
* In the same terminal, run `rosservice call enable_model_left buoy true` to enable the buoy model on the feed coming from the left camera. This model should now be publishing predictions
* To verify that the predictions are being published, you can run `rostopic list`, and you should see both `/camera/left/compressed` and `/cv/buoy/left` be listed. Then you can run `rostopic echo /cv/buoy/left` and the model predictions should be printed to the terminal

## Simulating Image feeds
`test_images.py` simulates the camera feed by constantly publishing images to a topic. It can publish still images, a folder of images, rosbag files, and video files to a topic on loop. This helps us locally test our code without needing to connect to a camera. Use `roslaunch` with the files when running.
* `test_images.launch`: Publishes dummy images to a topic every few seconds. It has the following three parameters:
* `feed_path`: Path to the image, video, folder, or rosbag file that you want published.
* `topic`: Name of topic that you want to be publish the images to. Required if `feed_path` is not a rosbag file.
* `framerate`: Number of frames published per second. Default value is set to 24. Used when `feed_path` is a folder or still image to determine rate at which to publish images.

Examples:

`roslaunch cv test_images.launch feed_path:=../assets/gate.mov topic:=/camera/left/compressed`: Runs test_images by taking gate.mov file in cv/assets and publishes the simulated image feed to the topic '/camera/left/compressed'.

`roslaunch cv test_images.launch feed_path:=../assets/buoy.jpg topic:=/camera/right/compressed framerate:=30`: Publishes the still image buoy.jpg in cv/assets to the topic '/camera/right/compressed' 30 times per second.

## Checking Camera Connection Status
`camera_test_connect.launch` starts the `connect_depthai_camera` and `connect_usb_camera` services which determine whether the stereo and mono cameras are connected. There are no arguments to the launch command.
* The `connect_depthai_camera` service has no arguments. A boolean `success` is returned indicating whether the connection was successful.
* The `connect_usb_camera` service requires a channel argument which is used to connect to the mono camera. A boolean `success` is returned indicating whether the connection was successful.

`ping_host.launch` starts the `ping_host` rosnode which regularly publishes a `DiagnosticArray` that contains log information from an attempted ping.

There are two arguments.
* `hostname` is the IP address of the host you want to ping. By default, hostname is `169.254.1.222` (the stereo camera IP address)
* `rate` specifies the time between each ping request in hertz. By default, the rate is 1 hz.

The `DianosticArray` that is pubished returns several values.
* `.header.stamp` contains the time of ping as a `rospy.Time` instance.
* `.status[].level` is 0 if the ping was successful, 2 otherwise.
* `.status[].name` is the `hostname` specified in the launch command.
* `.status[].stdout` is the full standard output log from the ping command.

# Other Files

The `utils.py` file contains the `DetectionVisualizer`class which provides functions to draw bounding boxes and their labels onto images. It is used by DepthAI files when publishing visualized detections.

The `image_tools.py` file contains the `ImageTools` class which provides functions to convert between OpenCV, ROS Image, and ROS CompressedImage formats. All scripts in this package use `ImageTools` to perform conversions between these types. `cv_bridge` is not used by any file or class in this package other than `ImageTools` itself.
