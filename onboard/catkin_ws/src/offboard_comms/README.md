# Offboard Communications Package

This package provides communications and functionality for Arduinos to be integrated with our main ROS system. The Arduinos handle thruster controls and pressure (depth) readings.

There are two Arduinos handled by the package, one for thrusters and one for the pressure sensor. The thruster Arduino runs a ROS node while the pressure Arduino dumps data over serial to the main computer, from which the `data_pub` node publishes the data to ROS. Supporting the thruster Arduino is `thrusters.cpp`, which maps thruster allocations to pulse widths sent to the thruster Arduino.

`offboard_comms` supports functionality for both a single Arduino and multiple Arduinos. Multiple Arduinos serve to support hardware that require different serial baud rates. This was necessitated by thruster publishers requiring the default 57600 baud while the pressure sensor is (from the factory) optimized for 9600 baud.

Multiple software serial ports is supported on some Arduinos, but launching both of these nodes independently is not feasable. Only one ROS node is currently being run as to avoid issue where the master ROS node loses sync with one or both of the Arduinos. Migrating to a RP2040 (Pico) was made, but proved cumbersome for programming purposes, so it was reverted to a Nano Every.

Oogway is currently configured to use two Arduinos, one for thrusters and one for the pressure sensor. However, functionality is still included for a single Arduino, as described below.

## Directory Structure

The notable files and folders in this package are as follows. Note how this specific directory structure is required per the upload scripts.

```
offboard_comms
├── CMakeLists.txt
├── package.xml
├── README.md
├── scripts
│   ├── ... Bash scripts for compiling and uploading Arduino code
├── launch
│   ├── serial.launch
│   ├── thrusters.launch
├── Arduino\ Sketchbook
│   ├── PressureArduino
│   │   ├── PressureArduino.ino # Arduino code for pressure sensor
│   │   ├── ... Libraries for pressure sensor
│   ├── ThrusterArduino
│   │   ├── ThrusterArduino.ino # Arduino code for thrusters
│   │   ├── ... Libraries for thrusters
├── data
│   ├── ... Thruster lookup tables
├── include
│   ├── thrusters.h
├── src
│   ├── thrusters.cpp
```

## Compile and Upload

### Upload script

On Linux hosts, with the container running in privileged mode, you may use the following command to compile and upload code to both Arduinos.
```
rosrun offboard_comms dual_upload.sh
```
Running just a single Arduino would require only a single Arduino to be connected over USB. Upload with the following command:
On Linux hosts, with the container running in privileged mode, you may use the following command to compile and upload code to both Arduinos.
```
rosrun offboard_comms dual_upload.sh
```
Running just a single Arduino would require only a single Arduino to be connected over USB. Upload with the following command:
```
rosrun offboard_comms arduino_upload.sh
```

Note that these commands requires the Arduino(s) to be an Arduino Nano using the old bootloader.

If the type of Arduino is changed, the `dual_upload.sh` (or `arduino_upload.sh` ) script will need to be updated to reflect the new board type. Specifically, `arduino-cli` installation and compliation calls.
Note that these commands requires the Arduino(s) to be an Arduino Nano using the old bootloader.

If the type of Arduino is changed, the `dual_upload.sh` (or `arduino_upload.sh` ) script will need to be updated to reflect the new board type. Specifically, `arduino-cli` installation and compliation calls.

To only compile (and not upload) the Arduino code, useful for testing builds, you may use:
```
rosrun offboard_comms dual_upload.sh -c
```

Or for a single Arduino:
```
rosrun offboard_comms dual_upload.sh -c
```

Or for a single Arduino:
```
rosrun offboard_comms arduino_upload.sh -c
```

Note that uploading to the Arduino might require restarting the Docker container. The same applies for the case where an Arduino is disconnected and reconnected.

To run successfully in `dual_upload.sh`, ensure that the Arduino serial numbers are accurately reflected in the script. These can be found by running `arduino-cli board list` with the Arduino connected.

Note that uploading to the Arduino might require restarting the Docker container. The same applies for the case where an Arduino is disconnected and reconnected.

To run successfully in `dual_upload.sh`, ensure that the Arduino serial numbers are accurately reflected in the script. These can be found by running `arduino-cli board list` with the Arduino connected.

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
roslaunch offboard_comms offboard_comms.launch
```
Now to test, start sending messages to the offboard device. For instance, to run all the thrusters at speed 0, you can use:
```
rostopic pub -r 20 /controls/thruster_allocs custom_msgs/ThrusterAllocs '{allocs: [0,0,0,0,0,0,0,0]}'
```
For testing on land, it is recommended to run the thrusters at speed 0.05:
```
rostopic pub -r 20 /controls/thruster_allocs custom_msgs/ThrusterAllocs '{allocs: [0.05,0.05,0.05,0.05,0.05,0.05,0.05,0.05]}'
```

## Topics and Services

### Thruster Allocations to PWMs
The node `thrusters.cpp` subscribes to `/controls/thruster_allocs` of type `custom_msgs/ThrusterAllocs`. This is an array of 64-bit floats, and they must be in range [-1, 1]. It also subscribes to `/sensors/voltage` of type `std_msgs/Float64`. This is a 64-bit float that is clamped to the range [14.0, 18.0].

The node maps the thruster allocations to pulse widths, accounting for the current system voltage, and sends them to the thruster Arduino. Note that this node runs _on the robot computer_, not the Arduino.

The node first loads 3 lookup tables containing pre-calculated information on the relation between force (given between the interval -1.0, 1.0), voltage (fixed by the lookup table, either 14.0v, 16.0v, or 18.0v), and PWM outputs. The tables are indexed by force, which is rounded to 2 decimal precision. These tables were computed from the Blue Robotics T200 Thruster performance data, found on [this page](https://bluerobotics.com/store/thrusters/t100-t200-thrusters/t200-thruster-r2-rp/) under "Technical Details".

For each recieved thruster allocation (force), the node first finds the closest force value (rounded to 2 decimal precision) in the lookup tables for the two voltages that bound the current voltage reading. Then, it performs linear interpolation between those two values using the current voltage to find the PWM that will result in the thruster exerting the desired force at the current voltage.

The PWMs are published to `/offboard/pwm` of type `custom_msgs/PWMAllocs`.

### Thruster Arduino
The thruster arduino subscribes to `/offboard/pwm` of type `custom_msgs/PWMAllocs`. This is an array of 16-bit unsigned integers specifying the pulse widths to use for each thruster's PWM. All messages must satisfy the following two conditions:
- The length of the array must match the number of thrusters on the robot
- Each value must be in range [1100, 1900]

Messages with incorrect length will be ignored. If any value is out of range, that thruster will be stopped. In both cases, an error message will be printed to the console.

Additionally, if it has been over 500 miliseconds since the last message was recieved, the thruster Arduino will stop all thrusters. This is to prevent the robot from continuing to move if controls is disabled or if the connection to the main computer is lost.

Note that an inaccuracy in the ESCs required adding a 31 microsecond offset to the PWM signal for Oogway.

```c++
#define THRUSTER_PWM_OFFSET 31 // Apply offset of +31 for oogway, remove for cthulhu
...
thrusters[i].write(pwm + THRUSTER_PWM_OFFSET);
```

This correction was determined to be a hardware defect with Oogway's Blue Robotics Basic ESCs. When sending a stop/configuration PWM signal of 1500 microseconds, the thrusters would interpret the command as a spin command. The introduced offset corrects for this issue.

### Pressure Arduino
The pressure arduino interprets the Blue Robotics Pressure Sensor using the MS5837 library. It sends the data over a serial line using each time the sensor gets a new reading. All of the processing for the depth data can be found in the `data_pub` package.