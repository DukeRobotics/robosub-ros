# Computer Vision

TODO: description

## Structure

`assets`: Folder with a dummy image to test the CV package on

`launch`: Contains the various launch files for our CV package. There is a general launch file for all the cameras (`cv.launch`), and then there are specific launch files for each camera (`cv_left`, `cv_right`, and `cv_down`). Finally, we have a launch file for our testing script `test_images.launch`

`models`: Contains our pre-trained models and a `.yaml` file that specifies the details of each model (classes predicted, topic name, and the path to the model weights)

`scripts`: This is the "meat" of our package. We have a detection script `detection.py` that will read in images and publish predictions onto a node. We also have a `test_images.py` script that is used for testing our package on a dummy video feed (basically one image repeated over and over). We can simulate different video feeds coming in on the different cameras on our `test_images.py` script.


## Setup

# Computer Vision

TODO: description

## Structure

TODO: describe scripts, messages in core, models, etc.

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

* Ensure 

mention default 
scp 

TODO: models.yaml, .pth files, download and move default model

## Testing

TODO: how to test in docker

## Listening

TODO

## Publishing

TODO




TODO: models.yaml, .pth files, download and move default model

## Testing

TODO: how to test in docker

## Listening

TODO

## Publishing

TODO


