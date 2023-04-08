# Offboard Communications Package

This package provides communications and functionality for an Arduino to be integrated with our main ROS system.

## Compile and Upload

### Upload script

On Linux hosts, with the container running in privileged mode, you may use the following command to compile and upload code to the Arduino.
```
rosrun offboard_comms arduino_upload.sh
```

Note that this requires the Arduino to be an Arduino Nano using the old bootloader.

To only compile (and not upload) the Arduino code, useful for testing builds, you may use:
```
rosrun offboard_comms arduino_upload.sh -c
```

### Manual Upload
We recommend using the upload script mentioned in the previous section, but the instructions are below for manual upload if that is preferred or required.

#### Generating Arduino libraries

In order to access some manner of ROS functionality inside the Arduino code, we first need to generate libraries that the Arduino can use. This includes both the ability to receive and publish messages and the format of those messages themselves. To do this run the command:
```
rosrun rosserial_arduino make_libraries.py .
```
This command will create a new `ros_lib` directory in your current directory that contains all of the code needed for the Arduino to talk to ROS. Adding the package name `offboard_comms` at the end tells rosserial to specifically also build the messages for the `offboard_comms` package (i.e. this package).

Whenever you make an update to the message types, you will need to re-run this command to regenerate the messages for Arduino.

#### Compiling and uploading the code
To actually get the code onto the Arduino, you need to install the newly generated `ros_lib` folder in your Arduino Libraries. To do this, go to your Arduino "sketchbook" folder (you can find this in preferences) and add `ros_lib` to the subfolder "libraries". More details at https://www.arduino.cc/en/hacking/libraries.

You can then use the ROS message types in Arduino code.

## Testing offboard communication
First set up the node on the computer that will talk to the Arduino.
```
roslaunch offboard_comms serial.launch
```
Now to test, start sending messages to the offboard device. For instance, to run all the thrusters at speed 0, you can use:
```
rostopic pub -r 10 /offboard/thruster_speeds custom_msgs/ThrusterSpeeds '{speeds: [0,0,0,0,0,0,0,0]}'
```
To set the first servo to a 90 degree angle, you can execute these commands in separate terminals:
```
rosrun offboard_comms servo_wrapper.py
rosservice call /offboard/set_servo_angle '{num: 0, angle: 90}'
```

## Topics and Services
### Thrusters
The thrusters are subscribed to the `/offboard/thruster_speeds` topic that is of type `custom_msgs/ThrusterSpeeds.msg`. This is an array of 8 signed 8-bit integers, which have range of [-128,127]. Negative values correspond to reverse (<1500 microseconds PWM), and positive values correspond to forward (>1500 microseconds PWM). 
### Servos
The servos utilize the `/offboard/set_servo_angle` service that is of type `custom_msgs/SetServo.srv`. The request to this service consists of an unsigned 8-bit integer `num` that corresponds to the pin number of the servo, and an unsigned 8 bit integer `angle` that corresponds to the desired angle (0-180). The reply consists of a bool `success` that corresponds to whether the request successfully set the angle.

Values for `num` are 0-indexed (meaning the first servo corresponds to pin number 0) and values that are greater than or equal to the number of servos will result in an unsuccessfull call. Values for `angle` that are >180 will also result in an unsuccessful call.

This service lives in the script `servo_wrapper.py` which communicates with the Arduino by publishing a message of type `custom_msgs/ServoAngleArray.msg`. This is an array of 8 8-bit unsigned integers, and publishes them to topic `/offboard/servo_angles`.
### Camera Relay
The camera relay is a hardware device that is used to kill and enable the power to the camera. The relay interrupts the POE power, forcing the camera to reboot when re-enabled. This is useful for when the camera is not responding to commands. This relay is connected to the onboard Arduino, and involves two topics and a service. The topics are `/offboard/camera_relay` and `/offboard/camera_relay_status`. The service is `rosservice call /enable_camera <true/false>`.

To set up the node on the computer that will interface with to the Arduino, run:
```
roslaunch offboard_comms serial.launch
```

Next, the Python code for the camera relay needs to be run. This is done by running the file `camera_hard_reset.py` in the `cv/scripts` folder. This file is set up to run automatically when the `camera_hard_reset.launch` file is run from within the `cv` package. To run this file, run:
```
roslaunch cv camera_hard_reset.launch
```
From this point, interfacing with the relay requires running:
```
rosservice call /enable_camera false
```
to disable the camera, and
```
rosservice call /enable_camera true
```
to re-enable the camera. By default, the camera is enabled. The camera relay topic can be checked by running:
```
rostopic echo /offboard/camera_relay
```
This will print the current state of the topic that the Arduino is subscribed to. The camera relay status topic can be checked by running:
```
rostopic echo /offboard/camera_relay_status
```
This will print the current state of the relay itself as reported by the Arduino. The distinction between the two topics is that the camera relay topic is the state that the relay is trying to be in, while the camera relay status topic is the state that the relay is actually in. The camera relay topic is set by the service call, while the camera relay status topic is set by the Arduino. The camera relay status topic is used to check if the camera is actually enabled or disabled, while the camera relay topic is used to check if the camera is trying to be enabled or disabled. Additionally, a message is displayed in the serial.launch terminal when the camera is enabled or disabled.

