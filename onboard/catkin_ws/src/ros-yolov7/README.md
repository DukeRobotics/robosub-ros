<div align="center">

# ROS package for official YOLOv7

![Ros](https://img.shields.io/badge/Ros-Noetic-informational?style=for-the-badge&logo=ROS)
![Python](https://img.shields.io/badge/Python-3.8-informational?style=for-the-badge&logo=Python&logoColor=FFFFFF)

</div>

> **Note** <br>
> This ROS noetic package is an implementation of the [official yolov7](https://github.com/WongKinYiu/yolov7), most credit goes to them.

<div align="center">
  <img src="docs/images/yolo_demo.gif" alt="Rviz" width="800"/>

  **Fig.1** RViz view of detection projection from a Gazebo Simulation
</div>

## :rocket: Installation from source

First you need to download all required source, like the [ros-image-transport-py](https://github.com/alexandrefch/ros-image-transport-py) package, that manage automaticly image stream.
Moreover you might need some python package like [OpenCV](https://github.com/opencv/opencv), you can install them by simply execute `pip install opencv-python` or better compile from source for GPU.

### :gear: Clone and Compile

<font size=2>

Clone thoose two packages inside the `src` directory of your catkin workspace.

```shell
git clone https://github.com/alexandrefch/ros-yolov7.git
git clone https://github.com/alexandrefch/ros-image-transport-py.git
```

</font>

Now you can compile them using your catkin tool (`catkin_make`, `catkin_build` ...).

## :clipboard: Configuration

### Weight

You can download some of the the YOLOv7 weights from the [official repository](https://github.com/WongKinYiu/yolov7) or use your own.

### Classes

Classes should be stored inside a simple text file where each single line represent a classe, like in the exemple below. 

<font size=2>

```
car
bus
person
bike
motorcycle
traffic light
truck
person
```

</font>

### Launch file

```xml
<?xml version="1.0"?>
<launch>
    <node pkg="yolov7" type="YoloV7.py" name="yolov7">
        <!-- Path to your weight -->
        <param name="weights_path" type="str" value="path_to_your/weights/weight.pt"/>
        <!-- Path to a class_labels.txt file, if you leave it empty then no class labels are visualized.-->
        <param name="classes_path" type="str" value="path_to_your/class_labels/labels.txt" />
        <!-- Input image topic name to subscribe to -->
        <param name="img_topic" type="str" value="/camera/image" />
        <!-- [optional]  Confidence threshold (default=0.25) -->
        <param name="conf_thresh" type="double" value="0.20" />
        <!-- [optional]  Intersection over union threshold (default=0.45) -->
        <param name="iou_thresh" type="double" value="0.45" />
        <!-- [optional]  Queue size for publishing (default=3) -->
        <param name="queue_size" type="int" value="1" />
        <!-- [optional] Image size to which to resize each input image before feeding into the network (the final output is rescaled to the original image size) (default=640) -->
        <param name="img_size" type="int" value="640" />
        <!-- [optional] Flag whether to also publish image with the visualized detections (default=false) -->
        <param name="visualize" type="bool" value="true" />
        <!-- [optional] Torch device 'cuda' or 'cpu' (default="cuda") -->
        <param name="device" type="str" value="cuda" />
        <!-- [optional] Node frequency (default=10) -->
        <param name="frequency" type="int" value="10" />
    </node>
</launch>
```

## :black_nib: Citation

<font size=2>

If you use this package in a scientific way or public way, don't forget to mention the official Yolov7 paper, and star their work on their [github repo](https://github.com/WongKinYiu/yolov7).

```
@article{wang2022yolov7,
  title={{YOLOv7}: Trainable bag-of-freebies sets new state-of-the-art for real-time object detectors},
  author={Wang, Chien-Yao and Bochkovskiy, Alexey and Liao, Hong-Yuan Mark},
  journal={arXiv preprint arXiv:2207.02696},
  year={2022}
}
```

</font>