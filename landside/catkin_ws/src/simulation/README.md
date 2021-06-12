# CoppeliaSim Simulation for the AUV
These are the instructions to get CoppeliaSim running on your own personal computer, connected to the ROS Topics inside a Docker container. The simulation currently reads a `custom_msgs/ThrusterSpeeds` of thruster values from the topic `/offboard/thruster_speeds`. This is expected to be an array of 8 ints, each from -128 to 128. This array should be in the order top-front-right, top-front-left, top-back-right, top-back-left, bottom-front-right, bottom-front-left, bottom-back-right, bottom-back-left.

Additionally, it publishes a [Odom](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html) to `/sensors/dvl/odom`, and an [Imu](http://docs.ros.org/noetic/api/sensor_msgs/html/msg/Imu.html) to `/sensors/imu/imu`.

## Installing the Simulation
### Assumptions
This readme assumes that you are running the Docker containers for [local testing](https://github.com/DukeRobotics/robosub-ros#local-testing).


### Downloading CoppeliaSim on your personal computer
First, download CoppeliaSim Edu for your OS version from [this link](https://coppeliarobotics.com/downloads). Then, follow the steps below for your specific OS.
#### Installing on Windows
Run the downloaded .exe file.

#### Installing on Ubuntu
Move the downloaded tar file to where you'd like the CoppeliaSim folder, and run `tar -kxvf [filename]`.

#### Installing on Mac
1. Unzip the download and move the resulting folder to Applications
2. To run, click on CoppeliaSim inside of that folder
3. If you get an error about models not found, run the following at the top of the CoppeliaSim folder
```bash
sudo xattr -r -d com.apple.quarantine *
```

## Running the Simulation

### Personal Computer Simulation Setup
1. Open CoppeliaSim.
2. Go to `File>Open Scene...` and open `empty_scene.ttt` in `personal/scenes`.
3. Press the play button to start the simulation. The robot should start bobbing up and down.

### Docker Simulation Setup
1. In the onboard container, make sure `roscore` is running.
2. Run `roslaunch simulation base_sim.launch` in the landside container. If you don't have any thruster speed publishing code and would just like to test basic movement in a square, instead run `roslaunch simulation test_sim_comm.launch`. Once `Starting main loop` is displayed, the simulation is ready to receive inputs.
3. Run whatever ROS topic publishing code you have. If you ran the test_sim_comm launch file, `scripts/square_command.py` will be executed, which publishes thruster values to naively make the robot move in a square (in reality, the robot will spin in wide circles because the robot isn't balanced).
4. The robot should start moving, if everything is successful.

#### A Note for Windows
On Windows machines with high res displays, some apps will not scale the font correctly. This happens because some applications essentially “tell” Windows that it will handle the scaling, but then don’t. To fix this, right click on the application icon, go to properties, compatibility, change high DPI settings, check override, and select system.
