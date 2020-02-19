# Compiling and deploying Arduino code

## Generating Arduino libraries
In order to access some manner of ROS functionality inside the Arduino code, we first need to generate libraries that the Arduino can use. This includes both the ability to receive and publish messages and the format of those messages themselves. To do this run the command:
```
rosrun rosserial_arduino make_libraries.py .
```
This command will create a new `ros_lib` directory in your current directory that contains all of the code needed for the Arduino to talk to ROS. Adding the package name `offboard_comms` at the end tells rosserial to specifically also build the messages for the `offboard_comms` package (i.e. this package).

Whenever you make an update to the message types, you will need to re-run this command to regenerate the messages for Arduino.

## Compiling and running the code
To actually get the code onto the Arduino, you need to install the newly generated `ros_lib` folder in your Arduino Libraries. To do this, go to your Arduino "sketchbook" folder (you can find this in preferences) and add `ros_lib` to the subfolder "libraries". More details at https://www.arduino.cc/en/hacking/libraries.

You can then use the ROS message types in Arduino code.

## Testing offboard communication
First set up the node on the computer that will talk to the Arduino. To do this, use `lsusb` and `sudo dmesg | grep tty` to determine the serial port on which the Arduino is connected. Then run (replace `/dev/ttyUSB0` with the serial port you find):
```
rosrun rosserial_python serial_node.py /dev/ttyUSB0
```
Now to test, start sending messages to the offboard device. For instance, to run all the thrusters at speed 0, you can use:
```
rostopic pub -r 10 /offboard/thruster_speeds offboard_comms/ThrusterSpeeds '{speeds: [0,0,0,0,0,0,0,0]}'
```