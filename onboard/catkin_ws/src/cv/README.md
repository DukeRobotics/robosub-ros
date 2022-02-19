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

Where `<camera>` is one of `left`, `right`, or `down`. 

After starting up a CV node, all models are initially disabled. You can select which model(s) you
want to enable for this camera by using the following service (where `<camera>` is the value you
chose above): 

* `enable_model_<camera>`
  * Takes in the model name (string) and a boolean flag to specify whether to turn the model on or off
  * Returns a boolean indicating whether the attempt was successful
  * Type: custom_msgs/EnableModel
  * E.g. `rosservice call enable_model_left buoy true` would enable the buoy model on the left camera feed
  
Once 1+ models are enabled for a specific node, they listen and publish to topics as described below in topics.

## Topics

#### Listening:

 * `/camera/<camera>/image_raw`
   * The topic that the camera publishes each frame to
   * If no actual camera feed is available, you can simulate one using `roslaunch cv test_images.launch`
   * Type: sensor_msgs/Image

#### Publishing:

* `<topic_name>/<camera>`
  * For each camera frame feed that a model processes, it will publish predictions to this topic  
  * `<topic_name>` is what was specified under `topic` in the `models.yaml` file for each enabled model
    (e.g. the example `buoy` model above might publish to `/cv/buoy/left`)
  * For each detected object in a frame, the model will publish the `xmin`, `ymin`, `xmax`, and `ymax` 
    coordinates (normalized to \[0, 1\], with (0, 0) being the top-left corner), `label` of the object, `score` (a confidence value in the range
    of \[0, 1\]), and the `width` and `height` of the frame. 
    * We apply nms to the model predictions before publishing. This removes predictions below a confidence threshold 
      and will remove overlapping predictions for the same class
  * If a model is enabled but detects no objects in a frame, it will publish a message with the label field set to 'none'
  * Type: custom_msgs/CVObject

Note that the camera feed frame rate will likely be greater than the rate at which predictions can 
be generated (especially if more than one model is enabled at the same time), so the publishing rate
could be anywhere from like 0.2 to 10 FPS depending on computing power/the GPU/other factors.  

## Structure

The following are the folders and files in the CV package:

`assets`: Folder with a dummy image to test the CV package on

`launch`: Contains the various launch files for our CV package. There is a general launch file for all the cameras (`cv.launch`), and then there are specific launch files for each camera (`cv_left`, `cv_right`, and `cv_down`). Finally, we have a launch file for our testing script `test_images.launch`

`models`: Contains our pre-trained models and a `.yaml` file that specifies the details of each model (classes predicted, topic name, and the path to the model weights)

`scripts`: This is the "meat" of our package. We have a detection script `detection.py` that will read in images and publish predictions onto a node. We also have a `test_images.py` script that is used for testing our package on a dummy video feed (basically one image repeated over and over). The path of this image is specified in `test_images.py`. We can simulate different video feeds coming in on the different cameras on our `test_images.py` script. The `utils.py` script has a function implementing the NMS algorithms; this goes through all raw predictions made by a model to remove overlapping predictions of the same class and to remove predictions below a confidence threshold.

`CMakeLists.txt`: A text file stating the necessary package dependencies and the files in our package.

`package.xml`: A xml file stating the basic information about the CV package

The CV package also has dependencies in the `core/catkin_ws/src/custom_msgs` folder.

## Examples
To simulate camera feed and then run a model on the feed from the left camera. We'll assume the model we want to run is called 'buoy':
* In one terminal, run `roslaunch cv test_images.launch` to start the script meant to simulate the raw camera feed
* In a new terminal, run `roslaunch cv cv_left.launch` to start the cv node
* In another new terminal, run `rosservice list` and should see the `enable_model_left` service be listed in the terminal
* In the same terminal, run `rosservice call enable_model_left buoy true` to enable the buoy model on the feed coming from the left camera. This model should now be publishing predictions
* To verify that the predictions are being published, you can run `rostopic list`, and you should see both `/camera/left/image_raw` and `/cv/buoy/left` be listed. Then you can run `rostopic echo /cv/buoy/left` and the model predictions should be printed to the terminal
