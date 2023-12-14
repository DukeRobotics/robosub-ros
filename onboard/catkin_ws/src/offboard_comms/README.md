# Offboard Communications Package

This package provides communications and functionality for Arduinos to be integrated with our main ROS system. The Arduinos handle thruster controls and pressure (depth) readings.

There are two Arduinos handled by the package, one for thrusters and one for the pressure sensor. The thruster Arduino runs a ROS node while the pressure Arduino dumps data over serial to the main computer, from which the `data_pub` node publishes the data to ROS.

`offboard_comms` supports functionality for both a single Arduino and multiple Arduinos. Multiple Arduinos serve to support hardware that require different serial baud rates. This was necessitated by thruster publishers requiring the default 57600 baud while the pressure sensor is (from the factory) optimized for 9600 baud.

Multiple software serial ports is supported on some Arduinos, but launching both of these nodes independently is not feasable. Only one ROS node is currently being run as to avoid issue where the master ROS node loses sync with one or both of the Arduinos. Migrating to a RP2040 (Pico) was made, but proved cumbersome for programming purposes, so it was reverted to a Nano Every.

The robot is currently configured to use two Arduinos, one for thrusters and one for the pressure sensor. However, functionality is still included for a single Arduino, as described below.

# Directory Structure

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
├── Arduino\ Sketchbook
│   ├── PressureArduino
│   │   ├── PressureArduino.ino # Arduino code for pressure sensor
│   │   ├── ... Libraries for pressure sensor
│   ├── ThrusterArduino
│   │   ├── ThrusterArduino.ino # Arduino code for thrusters
│   │   ├── ... Libraries for thrusters
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
roslaunch offboard_comms serial.launch
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
### Thrusters
The thrusters are subscribed to the `/controls/thruster_allocs` topic that is of type `custom_msgs/ThrusterAllocs.msg`. This is an array of 8 floats, which have range of [-1, 1]. Negative values correspond to reverse (<1500 microseconds PWM), and positive values correspond to forward (>1500 microseconds PWM).

Note that an inaccuracy in the ESCs required modifying a C++ file in the Blue Robotics library (`MultiplexedBasicESC.cpp`).

```c++
void MultiplexedBasicESC::write(int8_t speed){writeMicroseconds(map(speed, -128, 128, 1100, 1900)+31);}
//                                                                                               ^^^
// A 31 microsecond offset is added to the PWM signal to account for an inaccuracy in the ESCs
```

This correction was determined to be a hardware defect with Oogway's Blue Robotics Basic ESCs. When sending a stop/configuration PWM signal of 1500 microseconds, the thrusters would interpret the command as a spin command. The introduced offset corrects for this issue.

### Non-Linear Thruster Allocations
The script `thrusters.cpp` will automatically read updates from the onboard voltage sensor and perform linear interpolation given the voltage and desired thruster allocation (force) to compute the desired PWM allocation.

The script first pre-loads 3 lookup tables containing pre-calculated information on the relation between force (given between the interval -1.0, 1.0), voltage (fixed by the lookup table, either 14.0v, 16.0v, or 18.0v), and PWM outputs. The tables are indexed by force, which is rounded to 2 decimal precision.

### Pressure/Depth Sensor
The pressure arduino interprets the Blue Robotics Pressure Sensor using the MS5837 library. It sends the data over a serial line using each time the sensor gets a new reading. All of the processing for the depth data can be found in the `data_pub` package.