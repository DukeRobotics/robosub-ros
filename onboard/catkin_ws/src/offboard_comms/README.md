# Offboard Communications Package

This package provides communications and functionality for an Arduino to be integrated with our main ROS system.

## Generating Arduino libraries
In order to access some manner of ROS functionality inside the Arduino code, we first need to generate libraries that the Arduino can use. This includes both the ability to receive and publish messages and the format of those messages themselves. To do this run the command:
```
rosrun rosserial_arduino make_libraries.py .
```
This command will create a new `ros_lib` directory in your current directory that contains all of the code needed for the Arduino to talk to ROS. Adding the package name `offboard_comms` at the end tells rosserial to specifically also build the messages for the `offboard_comms` package (i.e. this package).

Whenever you make an update to the message types, you will need to re-run this command to regenerate the messages for Arduino.

## Compiling and uploading the code
To actually get the code onto the Arduino, you need to install the newly generated `ros_lib` folder in your Arduino Libraries. To do this, go to your Arduino "sketchbook" folder (you can find this in preferences) and add `ros_lib` to the subfolder "libraries". More details at https://www.arduino.cc/en/hacking/libraries.

You can then use the ROS message types in Arduino code.

## Testing offboard communication
First set up the node on the computer that will talk to the Arduino. To do this, use `lsusb` and `sudo dmesg | grep tty` to determine the serial port on which the Arduino is connected. Then run (replace `/dev/ttyUSB0` with the serial port you find):
```
rosrun rosserial_python serial_node.py /dev/ttyUSB0
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
