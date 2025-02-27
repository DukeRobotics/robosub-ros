# Duke Robotics Club - RoboSub ROS

![Build](https://github.com/DukeRobotics/robosub-ros/workflows/build/badge.svg)
![Docker Pulls](https://img.shields.io/docker/pulls/dukerobotics/robosub-ros)

We are a Duke University club competing in the RoboSub Competition. Check out [our website](https://duke-robotics.com).

> [!IMPORTANT]
> This repository, powered by [ROS Noetic](http://wiki.ros.org/noetic), was our primary codebase for RoboSub 2019 - RoboSub 2024. It is no longer maintained.
>
> We have moved to the [`robosub-ros2`](https://github.com/DukeRobotics/robosub-ros2) codebase, powered by [ROS2](https://github.com/ros2), for RoboSub 2025 and beyond.

## Introduction

This repo contains all of the code required for running and testing our robots.

Our code is Dockerized, making it straightforward to set up and run. See [Running Our Docker Images](#running-our-docker-images).

Once the containers are up and running, go to [Running Our Code](#running-our-code).

## Software Stack

Our stack is split into two workspaces -- onboard and landside -- each with corresponding Docker containers of the same name. Onboard consists of the code that runs on the robot's internal computer. Landside consists of the code that should be run on your local machine. The corresponding containers are connected via a local network and are configured to share ROS messages. For an in-depth explanation of the containers, see [docker](docker).

Our codebase is powered by the [Robot Operating System](https://www.ros.org) (ROS), and written primarily in Python.

The following components make up our software stack:

- Onboard:
    * [Acoustics](onboard/catkin_ws/src/acoustics) - Handles the task of locating an underwater hydrophone pinger.
    * [AVT Camera](onboard/catkin_ws/src/avt_camera) - Drives our ethernet cameras and publishes a live video feed.
    * [Controls](onboard/catkin_ws/src/controls) - Determines thruster outputs given a current and desired state.
    * [Computer Vision](onboard/catkin_ws/src/cv) - Locates objects (goals/obstacles) via camera input and machine learning.
    * [Data Pub](onboard/catkin_ws/src/data_pub) - Collects and parses data from sensors and publishes it for use by other packages.
    * [Execute](onboard/catkin_ws/src/execute) - Houses launch files that simplify starting and stopping our stack.
    * [Offboard Comms](onboard/catkin_ws/src/offboard_comms) - Allows communication between ROS code and the onboard Arduino.
    * [RoboSub Description](onboard/catkin_ws/src/robosub_description) - Contains files to describe and display our robot.
    * [Sensor Fusion](onboard/catkin_ws/src/sensor_fusion) - Interprets sensor data and publishes an estimation of the current robot state.
    * [Static Transforms](onboard/catkin_ws/src/static_transforms) - Makes static transforms available for other packages.
    * [System Utils](onboard/catkin_ws/src/system_utils) - Contains system helpers for evaluating performance and interacting with hardware.
    * [Task Planning](onboard/catkin_ws/src/task_planning) - Plans the tasks and motion of the robot by synthesizing inputs.
- Landside:
    * [Camera View](landside/catkin_ws/src/camera_view) - Package that allows for viewing, saving, and loading videos to simulate camera input.
    * [Joystick](landside/catkin_ws/src/joystick) - Allows manual joystick control for testing.
    * [Simulation](landside/catkin_ws/src/simulation) - Physics-enabled simulation that can be used for local testing.


## Flow

The high-level diagram below shows the components of the robot's `onboard` software that are used during the robot's autonomous operation and the flow of information between them. The arrows indicate the direction of data flow. The labels on each arrow indicate what data is transmitted. The components are color-coded to indicate their type.
- Sensors who feed data into the software are <span style="color: #c00">red</span>.
- Software packages in `onboard` are <span style="color: #00c">blue</span>.
- Hardware whose actions are controlled by the software are <span style="color: #080">green</span>.
- Hardware that serves as an intermediary between the software and other hardware is <span style="color: #990">yellow</span>.

```mermaid
flowchart TD
    IMU:::sensor --> VectorNav:::package
    DVL:::sensor --> DataPub[Data Pub]:::package
    PressureSensor[Pressure Sensor]:::sensor --> PressureArduino[Pressure Arduino]:::intermediateHardware
    Voltage[Voltage Sensor]:::sensor --> PressureArduino
    PressureArduino --> |Depth| DataPub
    PressureArduino --> |Voltage| DataPub
    VectorNav --> |Orientation| SensorFusion[Sensor Fusion]:::package
    VectorNav --> |Angular Velocity| SensorFusion
    DataPub --> |Linear Velocity| SensorFusion
    DataPub --> |Depth| SensorFusion
    FrontCamera[Front Camera]:::sensor --> CV[Computer Vision]:::package
    BottomCamera[Bottom Camera]:::sensor --> CV
    SensorFusion --> |State| TaskPlanning[Task Planning]:::package
    SensorFusion --> |State| Controls:::package
    CV --> |Object Detections| TaskPlanning
    Ping360:::sensor --> Sonar:::package
    Sonar --> |Object Poses| TaskPlanning
    Hydrophones:::sensor --> Acoustics:::package
    Acoustics --> |Pinger Positions| TaskPlanning
    TaskPlanning --> |Desired State| Controls
    TaskPlanning --> |Servo Commands| OffboardComms[Offboard Comms]:::package
    Controls --> |Thruster Allocations| OffboardComms
    DataPub --> |Voltage| OffboardComms
    OffboardComms --> |Pulse Widths| ThrusterArduino[Thruster Arduino]:::intermediateHardware
    OffboardComms --> |Servo Angles| ServoSensorArduino[Servo Sensor Arduino]:::intermediateHardware
    ThrusterArduino --> Thrusters:::outputs
    ServoSensorArduino --> MarkerDropperServo[Marker Dropper Servo]:::outputs

    classDef sensor fill:#c00;
    classDef package fill:#00c;
    classDef outputs fill:#080;
    classDef intermediateHardware fill:#990;

```

## Running Our Docker Images

### Required Software
1. Download and install the appropriate Docker client.

    * [Mac](https://docs.docker.com/docker-for-mac/install/)
    * [Windows (Home)](https://docs.docker.com/docker-for-windows/install-windows-home/)
    * [Windows (Pro, Education, etc.)](https://docs.docker.com/docker-for-windows/install/)<br>
        :information_source: Which Windows do I have? *Settings > System > About > look at "Edition"*

2. If you want graphics forwarding over SSH:

    * [Mac: XQuartz](https://www.xquartz.org/)
    * [Windows: MobaXterm](https://mobaxterm.mobatek.net/)<br>
        :warning: Open MobaXterm > click on Settings > go the X11 tab > uncheck RANDR. This allows us to use [rviz](http://wiki.ros.org/rviz) over SSH.

### Pool Testing
Use these instructions when running code on the robot itself.

1. Connect your computer to the robot via Ethernet. Then SSH into the robot's onboard computer.
    ```bash
    ssh robot@192.168.1.1
    ```
2. There, make sure the latest code from this repo is pulled. Then, run the onboard container if it's not already running.
    ```bash
    docker run -td --privileged --net=host -e ROBOT_NAME=oogway -v /home/robot/robosub-ros:/root/dev/robosub-ros -v /dev:/dev dukerobotics/robosub-ros:onboard
    ```
3. Clone this repo on your local computer. In the newly-created directory (robosub-ros), run the landside container. If on Windows, use PowerShell.
    ```bash
    docker run -td -p 2201:2201 -v ${PWD}:/root/dev/robosub-ros dukerobotics/robosub-ros:landside
    ```
4. SSH into the onboard container. Password is `robotics`.
    ```bash
    ssh -p 2200 root@192.168.1.1
    ```
5. SSH into the landside container. Password is `robotics`.
    ```bash
    ssh -XY -p 2201 root@localhost
    ```
6. Now go to [Running Our Code](#running-our-code).

### Local Testing
Use these instructions to test code on your computer by simulating the robot's execution. For all `docker compose` commands, replace `<robot name here>` with the robot name (`cthulhu` or `oogway`).

1. To run the containers, clone this repo. In the newly-created directory (`robosub-ros`), execute
    ```bash
    docker compose -f docker-compose-<robot name here>.yml up -d
    ```
    This will pull the images if you don't have them, create a new network that simulates the network we use on our robot, mount the code, and start the containers.

    To update the images, or to just pull them without running them, use `docker compose -f docker-compose-<robot name here>.yml pull`.

2. SSH into the onboard container. Password is `robotics`.
    ```bash
    ssh -p 2200 root@localhost
    ```
3. In a new tab, SSH into the landside container. Password is `robotics`.
    ```bash
    ssh -XY -p 2201 root@localhost
    ```
4. Now go to [Running Our Code](#running-our-code). Also set up our [simulation](landside/catkin_ws/src/simulation).
5. To stop and delete both containers and their network, in the `robosub-ros` directory, execute
    ```bash
    docker compose -f docker-compose-<robot name here>.yml down
    ```

## Running Our Code

:warning: All commands should be run inside of a Docker container.

### Building and Sourcing
The following needs to be done once at the beginning, and then later only when making larger structural changes to the workspace (such as adding a package).

#### Executing Build Script
We use a custom build script that simplifies the process of building and overlaying catkin workspaces.

To build our workspaces for ROS, in the `~/dev/robosub-ros` directory, execute
```bash
./build.sh <workspace>
```
where `<workspace>` is the workspace to build, either `onboard` or `landside`.

#### Sourcing Setup File
Once the build script has finished executing, source the setup file using
```bash
source <workspace>/catkin_ws/devel/setup.bash
```
where `<workspace>` is the workspace you just built. This will also be the last line that is printed from the build script, so you can also just copy and execute that.

You should now be ready to use our packages and ROS.

### Running Launch Files
Here are some common launch configurations for both pool and local testing.

* Test controls using `test_state_publisher.py`
    ```bash
    roslaunch execute motion.launch
    rosrun controls test_state_publisher.py
    rosservice call /enable_controls true
    ```

    but change the first line for local testing in the simulation
    ```bash
    roslaunch execute motion.launch sim:=true
    ```
* Useful commands for testing
    * Stop all thrusters
        ```bash
        rosservice call /enable_controls false
        ```
    * Echo current state
        ```bash
        rostopic echo /state
        ```
    * Reset state to zero
        ```bash
        rosservice call /set_pose  # tab complete, then change w from 0.0 to 1.0
        ```

        or here's the full command for copy-pasting:
        ```bash
        rosservice call /set_pose "pose:
            header:
                seq: 0
                stamp:
                    secs: 0
                    nsecs: 0
                frame_id: ''
            pose:
                pose:
                    position: {x: 0.0, y: 0.0, z: 0.0}
                    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
                covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
        ```

### Cleaning Build
To clean the build outputs from a workspace (build, devel, and logs folders), in the `~/dev/robosub-ros` directory, execute
```bash
./build.sh clean <workspace>
```
where `<workspace>` is either onboard or landside. If you would like to clean all workspaces, then you may simply execute
```bash
./build.sh clean
```

## Contributing

To contribute to this repository, see [CONTRIBUTING.md](CONTRIBUTING.md).
