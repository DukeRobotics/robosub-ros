# Computer Vision

The computer vision package listens for images/frames coming from 3 different cameras: left, right, and down. The package 
will then run pre-trained machine learning models on each frame and output bounding boxes for the various objects 
in the frame. These objects could be the gate, buoys, etc. The package will publish to different topics depending 
on which models are enabled and which cameras are being used.

## Setup

Generally, you would train a separate object detection model for each task you need computer vision for (gates, buoys, etc.). You can then load them as follows:

* Create object detection models and save them as .pth files (see [here](https://github.com/DukeRobotics/documentation/tree/master/cv/training))
* Place these models in the `/models` folder
* Update the `/models/models.yaml` file with each model's details in the following format:

```yaml
<model_name>:  # A name/identifier for your model
  classes: [<class1>, <class2>, ...]  # The classes the model is trained to predict
  topic: <topic_name>  # the base topic name your predictions will be published to
  weights: <file_name>  # the name of your model file
...
```

Example entry for a buoy model:

```yaml
buoy:
  classes: [alien, bat, witch]
  topic: /cv/buoy
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
roslaunch cv cv_<camera>.launch
```

Where `<camera>` is one of `left`, `right`, `down`, or `stereo` (discussed below). 

After starting up a CV node, all models are initially disabled. You can select which model(s) you
want to enable for this camera by using the following service (where `<camera>` is the value you
chose above): 

* `enable_model_<camera>`
  * Takes in the model name (string) and a boolean flag to specify whether to turn the model on or off
  * Returns a boolean indicating whether the attempt was successful
  * Type: custom_msgs/EnableModel
  
Once 1+ models are enabled for a specific node, they listen and publish to topics as described below.

## Topics

#### Listening:

 * `/camera/<camera>/image_raw`
   * The topic that the camera publishes each frame to
   * If no actual camera feed is available, you can simulate one using `roslaunch cv test_images.launch`
   * Type: sensor_msgs/Image
   * Note: If `stereo` is chosen for the camera, both the `left` and `right` cameras will be listened to.

#### Publishing:

* `<topic_name>/<camera>`
  * For each camera frame feed that a model processes, it will publish predictions to this topic  
  * `<topic_name>` is what was specified under `topic` in the `models.yaml` file for each enabled model
    (e.g. the example `buoy` model above might publish to `/cv/buoy/left`)
  * For each detected object in a frame, the model will publish the `xmin`, `ymin`, `xmax`, and `ymax` 
    coordinates (normalized to \[0, 1\], with (0, 0) being the top-left corner), `label` of the object, `score` (a confidence value in the range
    of \[0, 1\]), and the `width` and `height` of the frame. If `stereo` is chosen, additional fields will be published (discussed below).
    * Note: Only the highest-confidence prediction of each label type is published (e.g. if 5 bounding boxes 
      were predicted for a gate object, only the one with the highest score is chosen)
  * If a model is enabled but detects no objects in a frame, it will publish a message with the label field set to 'none'
  * Type: custom_msgs/CVObject

Note that the camera feed frame rate will likely be greater than the rate at which predictions can 
be generated (especially if more than one model is enabled at the same time), so the publishing rate
could be anywhere from 0.2 to 10 FPS depending on computing power/the GPU/other factors.  


## Structure

The following are the folders and files in the CV package:

`assets`: Folder with a dummy image to test the CV package on.

`data`: Folder with calibration files required for stereo vision (discussed below).

`launch`: Contains the various launch files for our CV package. There is a general launch file for all the cameras (`cv.launch`), 
and then there are specific launch files for each camera (`cv_left`, `cv_right`, and `cv_down`, plus `cv_stereo`). Finally, 
we have a launch file for our testing script `test_images.launch`.

`models`: Contains our pre-trained models and a `.yaml` file that specifies the details of each model (classes predicted, topic name, 
and the path to the model weights).

`scripts`: This is the "meat" of our package. We have a detection script `detection.py` that will read in images and publish 
predictions onto a node. We also have a `test_images.py` script that is used for testing our package on a dummy video feed 
(basically 1-2 images repeated over and over). We can simulate different video feeds coming in on the different cameras on 
our `test_images.py` script.

`CMakeLists.txt`: A text file stating the necessary package dependencies and the files in our package.

`package.xml`: A xml file stating the basic information about the CV package

The CV package also has dependencies in the `core/catkin_ws/src/custom_msgs` folder.


## Stereo Vision

When creating a CV node, you also have the option to specify `stereo` instead of `left`, `right`, or `down`. 
This will cause the CV node to listen to both the `left` and `right` images, and for each CVObject it publishes,
it will also include the following additional fields: 

* `stereo_enabled`: whether stereo information is available (true if camera is `stereo`, false otherwise)
* `depth`: the estimated z distance to the object
* `distance` the estimated total distance to the object (combined x, y, and z distance)
* `angle_horiz`: the horizontal angle of the object relative to the camera in degrees (left to right, 0 is straight forward)
* `angle_vert`: the vertical angle of the object relative to the camera in degrees (bottom to top, 0 is straight forward)
  
Note: These fields are included in every CVObject, but they are calculated and set only when `stereo_enabled` is set to true.


### Setup

In order to perform stereo vision, some additional files need to be included in the `data` folder:

* `fov_data.npy`: The field of view of the cameras in degrees
  * This is a NumPy array of length 2 (horizontal FOV then vertical FOV) saved via `np.save`
  * These values are used to calculate the angle of objects relative to the camera
  * e.g. `np.array([75.8, 36.2])`
* `remap_matrices.npy`: The undistortion and rectification matrices 
  * This is a saved 2x2 NumPy array where each value is one of the left/right undistortion/rectification matrices 
  * The two matrices per each left/right camera are computed by a call to `cv2.initUndistortRectifyMap` during the stereo calibration process
  * These matrices are used to undistort the camera feed images and thus allow a well-calibrated disparity map to be generated  
* `q_matrix.npy`: The disparity to depth map matrix
  * This is a saved 4x4 NumPy array
  * The matrix is computed by a call to `cv2.stereoRectify`
  * This is used to reproject a computed disparity map into a 3D depth map from which depth/distance can be extracted

A change to the camera specs or the angle/distance between the left and right cameras will require these matrices to be recalculated. 
