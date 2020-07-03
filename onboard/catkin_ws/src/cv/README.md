# Computer Vision

The computer vision package listens for images coming from 3 different cameras: left, right, and down. The package will then run a pre-trained machine learning model on the image and output bounding boxes for the various objects in the frame. These objects could be the gate, buoys, etc. The package will publish on different topics depending on which camera's image is being analyzed.

## Structure

The following are the folders and files in the CV package.

`assets`: Folder with a dummy image to test the CV package on

`launch`: Contains the various launch files for our CV package. There is a general launch file for all the cameras (`cv.launch`), and then there are specific launch files for each camera (`cv_left`, `cv_right`, and `cv_down`). Finally, we have a launch file for our testing script `test_images.launch`

`models`: Contains our pre-trained models and a `.yaml` file that specifies the details of each model (classes predicted, topic name, and the path to the model weights)

`scripts`: This is the "meat" of our package. We have a detection script `detection.py` that will read in images and publish predictions onto a node. We also have a `test_images.py` script that is used for testing our package on a dummy video feed (basically one image repeated over and over). We can simulate different video feeds coming in on the different cameras on our `test_images.py` script.

`CMakeLists.txt`: A text file stating the necessary package dependencies and the files in our package.

`package.xml`: A xml file stating the basic information about the CV package

The CV package also has dependencies in the `core/catkin_ws/src/custom_msgs` folder.


## Setup

* Create object detection models and save them as .pth files (see [here](https://github.com/DukeRobotics/robosub-cv/tree/master/training))
* Place these models in the `/models` folder
* Update the `/models/models.yaml` file with your model details in the following format:

```yaml
model_name:  # A name for your model
  classes: [class1, class2, ...]  # The classes the model is trained to predict
  topic: /cv/model_name  # the base topic name your predictions will be published to
  weights: buoy.pth  # the relative path to your model file
```

Note: To get the model files onto the docker container, you may have to use `scp`. Also, if you come across the following error: 

`URLError: <urlopen error [Errno -3] Temporary failure in name resolution>`

Navigate to [this url](https://download.pytorch.org/models/fasterrcnn_resnet50_fpn_coco-258fb6c6.pth) 
to manually download the default model file used by the Detecto package. Move this file onto the Docker
container under the directory `/root/.cache/torch/checkpoints/` (do not rename the file). 


## Testing

The following test plan walks through how to launch a camera node which takes in fake image data and publishes bounding box predictions to a separate topic.

1. Kick off one of the camera launch files. This will start up the detection node. Since we have three camera launch files, any of the three launch files ending in  *_right*, *_left*, or *_bottom* can be used. 
 `roslaunch cv cv_left.launch`
2. Kick off the test_images launch file. This will start up the test_images node and simulate a camera feed by repeatedly publishing a test image to the camera topic.  
 `roslaunch cv test_images.launch`  
3. Ensure that the detection node is working properly. First, view the list of active topics via
`rostopic list` 
Output the contents of the topic via
`rostopic echo /cv/buoy/left` (or other topic name depending on models.yaml and camera)
Ensure that correct coordinates are being published.

## Topics 

### Listening

 - ```'{}/{}'.format(model['topic'], camera)```
 The topic that stores information about an object's bounding box
   + model['topic'] is the path to the topic stored in ```models.yaml```
   + camera is the ```'~camera'``` parameter 
   + It contains 4 float64s representing the coordinates of the bounding box, 1 float64 representing the accuracy of the prediciton, and a String representing the name of the image.
   + Type: custom_msgs/CVObject

### Publishing

 - ```'/camera/{}'.format(CAMERA)```
 The topic that the camera publishes its feed to
   + ```CAMERA``` represents the camera the image is received from (ex: left)
   + An image should be published to this topic
     + The image must be processed using ``.cv2_to_imgmsg(image, 'bgr8')``` 
       + Make sure you import CvBridge for the above method (```from cv_bridge import CvBridge```)
   + Type: sensor_msgs/Image
