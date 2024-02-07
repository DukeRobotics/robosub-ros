# Offboard Communications Package

This package provides communications and functionality for Arduinos to be integrated with our main ROS system. The Arduinos handle thruster controls, pressure (depth), and voltage readings.

There are two Arduinos handled by the package, one for thrusters and one for the pressure sensor. The thruster Arduino runs a ROS node while the pressure Arduino dumps data over serial to the main computer, from which the `data_pub` package publishes the data to ROS. Supporting the thruster Arduino is `thrusters.cpp`, which maps thruster allocations to pulse widths sent to the thruster Arduino.

`offboard_comms` supports functionality for both a single Arduino and multiple Arduinos. Multiple Arduinos serve to support hardware that require different serial baud rates. This was necessitated by thruster publishers requiring the default 57600 baud while the pressure sensor is factory-optimized for 9600 baud.

Multiple software serial ports is supported on some Arduinos, but launching both of these nodes independently is not feasable. Only one ROS node is currently being run as to avoid issue where the master ROS node loses sync with one or both of the Arduinos. Migration to a RP2040 (Pico) was attempted, but proved cumbersome for programming purposes, so it was reverted to a Nano Every.

Oogway is currently configured to use two Arduinos, one for thrusters and one for the pressure and voltage sensors. However, functionality is still included for a single Arduino, as described below.

## Directory Structure

The notable files and folders in this package are as follows. Note how this specific directory structure is required per the upload scripts.

```
offboard_comms
├── Arduino Sketchbooks
│   ├── PressureArduino
│   │   ├── PressureArduino.ino  # Arduino code for pressure and voltage sensors
│   │   ├── ... Libraries for pressure sensor
│   ├── ThrusterArduino
│   │   ├── ThrusterArduino.ino  # Arduino code for thrusters
│   │   ├── ... Libraries for thrusters
│   ├── cthulhuThrusterOffset.h  # PWM offset for Cthulhu thrusters
│   ├── oogwayThrusterOffset.h  # PWM offset for Oogway thrusters
├── config
│   ├── arduino.yaml # Arduino config file
├── data
│   ├── ... Thruster lookup tables
├── include
│   ├── thrusters.h
├── launch
│   ├── offboard_comms.launch  # Primary launch file; includes serial.launch and thrusters.launch
│   ├── serial.launch  # Start thruster Arduino ROS node
│   ├── thrusters.launch  # Start thruster allocs to PWM conversion node
├── scripts
│   ├── arduino.py  # CLI to install libraries, find ports, compile, and upload Arduino code
│   ├── copy_offset.sh  # Bash script to copy the correct PWM offset file to the Thruster Arduino sketchbook
│   ├── servo_wrapper.py  # ROS node to publish requests from ROS service calls to a ROS topic
├── src
│   ├── thrusters.cpp  # ROS node to convert thruster allocations to PWMs
├── CMakeLists.txt
├── package.xml
├── README.md
```

## Arduino Config

The `config/arduino.yaml` file is used to specify the Arduinos to be used. The file contains a set of Arduino names, each with a dictionary of properties. The properties are used to find the port that the Arduino is connected to, install the required libraries, compile the Arduino code, and upload the code to the Arduino. The file is structured as follows:
```yaml
arduino_name_1:
    ftdi: arduino_1_ftdi_string
    fqbn: arduino_1_fqbn
    core: arduino_1_core
    sketch: arduino_1_path_to_sketch_relative_to_offboard_comms
    requires_ros_lib: boolean_true_or_false
    (optional) pre_compile: arduino_1_pre_compile_command
    (optional) post_compile: arduino_1_post_compile_command
arduino_name_2:
    ...
```
The structure shown for `arduino_name_1` is repeated for all Arduinos.

Below are more details on the keys in the dictionary:
- `arduino_name_1`, `arduino_name_2`, etc. are the names of the Arduinos. These names are used to refer to the Arduinos in the [CLI](#command-line-interface) and in other files. They can be any string, but should be descriptive of the Arduino. They must be unique. `all` is a special name used by the CLI to refer to all Arduinos; do **_not_** use it as a top-level key in this file. The names do not necessarily correspond to any name recognized by the Arduino CLI or operating system.
- `ftdi` is the FTDI string of the Arduino. This is used to find the port that the Arduino is connected to. To obtain the FTDI string, run
    ```bash
    ls /dev/serial/by-id
    ```
    and find the string corresponding to the desired Arduino. The FTDI string is the set of characters after the last underscore `_` but before the hyphen `-`. For example, in the following output,
    ```
    usb-Arduino_LLC_Arduino_Nano_Every_E49AFA8B51514C4B39202020FF024242-if00
    ```
    the FTDI string is `E49AFA8B51514C4B39202020FF024242`.
- `fqbn` is the fully qualified board name of the Arduino. This is used when compiling and uploading the Arduino code. It is the string that appears in the output of `arduino-cli board list` under the FQBN column.
- `core` is the name of the Arduino core that the Arduino uses; the core library will be installed before compiling and uploading code. It is the string that appears in the output of `arduino-cli board list` under the core column.
- `sketch` is the path to the directory containing the Arduino sketch, relative to the `offboard_comms` package. The specified directory must contain a `.ino` file. This is the sketch that is compiled and uploaded to the Arduino. It must **_not_** include a leading `/` or `./`.
- `requires_ros_lib` is a boolean that specifies whether the Arduino requires the ROS library. If `true`, the ROS library will be installed using the Arduino CLI before compiling code. If `false`, it will not be installed. The ROS library is required for the Arduino to communicate with ROS. If the Arduino does not communicate with ROS, this should be `false`.
- `pre_compile` is an optional bash command to run _before_ compiling the Arduino code. It is useful for running any commands that are required before compiling the Arduino code, such as installing libraries or modifying files. If an Arduino does not require a pre compile command, this key must **_not_** be present under the Arduino's dictionary.
- `post_compile` is an optional bash command to run _after_ compiling the Arduino code. It is useful for running any commands that are required after compiling the Arduino code, such as deleting temporary files. If an Arduino does not require a post compile command, this key must **_not_** be present under the Arduino's dictionary.


## Arduino Compile and Upload

### Command Line Interface

On Linux hosts, with the container running in privileged mode, use the CLI provided by `arduino.py` to install libraries, find ports, compile, and upload Arduino code. The CLI is a wrapper around the Arduino CLI and other commands, and is used to simplify the process of uploading code to the Arduino. The CLI _requires_ a properly formatted `config/arduino.yaml` file to run properly (see [Arduino Config](#arduino-config)).

The CLI is run as follows:
```
rosrun offboard_comms arduino.py <COMMAND> <ARDUINO_NAME_1> <ARDUINO_NAME_2> ... <OPTIONAL_FLAGS>
```

`<COMMAND>` must be one of the following:
- `install-libs`: Install the Arduino core libraries and for the specified Arduino(s). Also installs the ROS library if required by one or more of the specified Arduino(s).
- `find-ports`: Find the ACM port(s) for the specified Arduino(s).
- `compile`: Install all required libraries and compile the sketch(es) for the specified Arduino(s).
- `upload`: Install all required libraries, compile the sketch(es), and upload the sketch(es) to the specified Arduino(s).

Each `<ARDUINO_NAME>` must be a top-level key in `config/arduino.yaml`, or `all`. If `all` is specified, the command will be run on all Arduinos in `config/arduino.yaml`.

The following optional flags can be added to the end of the command:
- `-p`, `--print-output`: Print the output of the commands being run by the CLI. This is useful for debugging. It is available with the `install-libs`, `compile`, and `upload` commands.
- `-nl`, `--no-linebreaks`: Do not print any line breaks, labels, or prefixes; only print the port(s) found. This is available with the `find-ports` command. It is useful for running the command in a script or for piping the output to another command.

For example, to upload the sketches for all Arduinos, run:
```bash
rosrun offboard_comms arduino.py upload all
```
To find the ports for all Arduinos, run:
```bash
rosrun offboard_comms arduino.py find-ports all
```
To install the libraries for `arduino_name_1` and `arduino_name_2`, run:
```bash
rosrun offboard_comms arduino.py install-libs arduino_name_1 arduino_name_2
```
To compile the sketch for `arduino_name_1` and print output run:
```bash
rosrun offboard_comms arduino.py compile arduino_name_1 -p
```

Note that uploading to the Arduino might require restarting the Docker container, especially when one or more Arduinos have been disconnected and reconnected since the Docker container was started.

All output from the CLI printed to the console will be prefixed with `Arduino.py:`. This is useful for distinguishing the output of the CLI from the output of other commands.

Additionally, the CLI also prints all commands being run to the console. These commands are prefixed with `CMD:` (in addition to the general prefix above). This is useful for distinguishing the commands being run by the CLI from other output.

### Manual Upload

It is strongly suggested to use the CLI mentioned in the previous section, but instructions for manual upload are below if that is preferred or required.

#### Generating ROS Arduino libraries

In order to access some manner of ROS functionality inside the Arduino code, we first need to generate libraries that the Arduino can use. This includes both the ability to receive and publish messages and the format of those messages themselves. To do this run the command:
```
rosrun rosserial_arduino make_libraries.py .
```
This command will create a new `ros_lib` directory in your current directory that contains all of the code needed for the Arduino to talk to ROS. Adding the package name `offboard_comms` at the end tells rosserial to specifically also build the messages for the `offboard_comms` package (i.e. this package).

Whenever you make an update to the message types, you will need to re-run this command to regenerate the messages for Arduino.

#### Installing the ROS Arduino libraries
To actually get the code onto the Arduino, you need to install the newly generated `ros_lib` folder in your Arduino Libraries. To do this, go to your Arduino "sketchbook" folder (you can find this in preferences) and add `ros_lib` to the subfolder "libraries". More details at https://www.arduino.cc/reference/en/libraries/.

You can then use the ROS message types in Arduino code.

#### Compiling and Uploading
To compile and upload the Arduino code, you can use the [Arduino CLI](https://arduino.github.io/arduino-cli/0.35/) or the Arduino IDE installed on the robot.

## Thruster Allocations to PWMs
The node `thrusters.cpp` subscribes to `/controls/thruster_allocs` of type `custom_msgs/ThrusterAllocs`. This is an array of 64-bit floats, and they must be in range [-1, 1]. It also subscribes to `/sensors/voltage` of type `std_msgs/Float64`. This is a 64-bit float that is clamped to the range [14.0, 18.0].

The node maps the thruster allocations to pulse widths, accounting for the current system voltage, and sends them to the thruster Arduino. Note that this node runs _on the robot computer_, not the Arduino.

The node first loads 3 lookup tables containing pre-calculated information on the relation between force (given between the interval -1.0, 1.0), voltage (fixed by the lookup table, either 14.0v, 16.0v, or 18.0v), and PWM outputs. The tables are indexed by force, which is rounded to 2 decimal precision. These tables were computed from the Blue Robotics T200 Thruster performance data, found on [this page](https://bluerobotics.com/store/thrusters/t100-t200-thrusters/t200-thruster-r2-rp/) under "Technical Details".

For each recieved thruster allocation (force), the node first finds the closest force value (rounded to 2 decimal precision) in the lookup tables for the two voltages that bound the current voltage reading. Then, it performs linear interpolation between those two values using the current voltage to find the PWM that will result in the thruster exerting the desired force at the current voltage.

The PWMs are published to `/offboard/pwm` of type `custom_msgs/PWMAllocs`.

## Thruster Arduino
The thruster arduino subscribes to `/offboard/pwm` of type `custom_msgs/PWMAllocs`. This is an array of 16-bit unsigned integers specifying the pulse widths to use for each thruster's PWM. All messages must satisfy the following two conditions:
- The length of the array must match the number of thrusters on the robot
- Each value must be in range [1100, 1900]

Messages with incorrect length will be ignored. If any value is out of range, that thruster will be stopped. In both cases, an error message will be printed to the console.

Additionally, if it has been over 500 miliseconds since the last message was recieved, the thruster Arduino will stop all thrusters. This is to prevent the robot from continuing to move if controls is disabled or if the connection to the main computer is lost.

### ESC Offset
Note that an inaccuracy in the ESCs required adding a 31 microsecond offset to the PWM signal for Oogway. This correction was determined to be a hardware defect with Oogway's Blue Robotics Basic ESCs. When sending a stop/configuration PWM signal of 1500 microseconds, the thrusters would interpret the command as a spin command. The introduced offset corrects for this issue.

This offset is set in a header file corresponding to each robot, with file name `<ROBOT_NAME>ThrusterOffset.h` where `<ROBOT_NAME>` is the value of the `ROBOT_NAME` enviornment variable. The offset is added to the PWM signal in the Arduino code.

Before compiling the thruster Arduino sketch, the `copy_offset.sh` script copies the correct header to the thruster Arduino sketchbook and renames it to `offset.h`. After compilation is complete, the `copy_offset.sh` script deletes `offset.h` from the sketchbook.

## Testing Thrusters
First start the ROS nodes for the thruster Arduino and thruster allocs to PWMs:
```
roslaunch offboard_comms offboard_comms.launch
```
Now to test, start sending thruster allocs messages. For instance, to set 0 allocs for all thrusters:
```
rostopic pub -r 20 /controls/thruster_allocs custom_msgs/ThrusterAllocs '{allocs: [0,0,0,0,0,0,0,0]}'
```
For testing on land, it is recommended to set 0.05 allocs for all thrusters:
```
rostopic pub -r 20 /controls/thruster_allocs custom_msgs/ThrusterAllocs '{allocs: [0.05,0.05,0.05,0.05,0.05,0.05,0.05,0.05]}'
```

## Pressure Arduino

The Pressure Arduino primarily publishes depth data over serial. It also is meant to support additional sensors, as is the case with the voltage sensor.

The `data_pub` package is responsible for parsing the data published to serial and publishing the data to ROS.

The different readings are intermixed from the different sensors. Each sensor type has a corresponding header:

- `P`: Pressure
- `V`: Voltage

Example data from the Arduino is as follows:
```
P:0.73
V:15.63
P:0.73
P:0.73
P:0.73
P:0.73
P:0.73
P:0.73
P:0.73
P:0.73
P:0.73
P:0.74
P:0.74
P:0.74
P:0.74
P:0.74
P:0.73
P:0.74
P:0.74
P:0.73
P:0.74
P:0.74
V:15.67
P:0.73
P:0.74
P:0.74
```

### Pressure

The pressure arduino interprets the Blue Robotics Pressure Sensor using the MS5837 library. It sends the data over a serial line using each time the sensor gets a new reading, so the rate is not defined other than "as fast as possible." All of the processing for the depth data can be found in the `data_pub` package.

After extensive testing, it was found that proper functioning of the pressure sensor requires two key things:
- The most up-to-date Arduino Wire library, which includes the [`setWireTimeout` function](https://www.arduino.cc/reference/en/language/functions/communication/wire/setwiretimeout/). As of February 2024, this is **_not_** available on Arduinos using MegaAVR 1.8.3. Therefore, an AVR-based Arduino is required (e.g. Nano or Uno).
- Consistent voltage of 5V. The sensor is factory-optimized for 5V, so readings will be inaccurate if the voltage is not 5V. Experimental results have shown that when the voltage is dropped to at or below 2.5V, the sensor will stop responding to I2C requests.

If these conditions are met, the sensor will return accurate readings and will not become unresponsive. If the sensor does become unresponsive, a 100 milisecond timeout has been set to prevent the Arduino from hanging.

The `MS5837` library has been modified to handle errors in reading the sensor.

The `MS5837::read` function has been modified to return a byte indicating the success/error of the reading. If any call to `Wire::endTransmission` returns a non-zero value, the function will return that value. If any call to `Wire::requestFrom` times out, the function will return 5. If the function returns 0, the reading was successful.

Below is the list of error codes and their meanings:
- 0: success.
- 1: data too long to fit in transmit buffer.
- 2: received NACK on transmit of address.
- 3: received NACK on transmit of data.
- 4: other error.
- 5: timeout

The list is identical to the [one provided by the `Wire::endTransmission` function](https://www.arduino.cc/reference/en/language/functions/communication/wire/endtransmission/).

If a non-timeout error occurs, no data is published to serial. On the next iteration of the loop, reading will be attempted again. If a timeout error occurs, an attempt to reinitialize the sensor will be made in every iteration of the loop until it is successful.

Errors in reading the pressure sensor do not affect the voltage sensor. Voltage readings will continue to be published even if the pressure sensor is unresponsive.

### Voltage

Voltage publishing over serial is also handled by the Pressure Arduino. The voltage is published as a float over serial, and is published at 1 Hz.

The votage is calibrated based on the onboard Arduino's voltage. This is important as the used voltage sensor requires knowledge of its own voltage as it uses a voltage divider to measure the voltage. The voltage sensor used is the a generic voltage sensor.