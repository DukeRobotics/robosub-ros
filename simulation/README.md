# CoppeliaSim Simulation for the AUV
These are the instructions to get CoppeliaSim running on your own personal computer, connected to the ROS Topics inside a Docker container. The simulation currently reads a [Float32MultiArray](http://docs.ros.org/melodic/api/std_msgs/html/msg/Float32MultiArray.html) of thruster values from the topic /sim/move. This is expected to be an array of 8 floats, each from -1 to 1. This array should be in the order top-front-right, top-front-left, top-back-right, top-back-left, bottom-front-right, bottom-front-left, bottom-back-right, bottom-back-left.

Additionally, it publishes a [TwistStamped](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/TwistStamped.html) to /sim/dvl, a [PoseStamped](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PoseStamped.html) to /sim/pose, and an [Imu](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Imu.html) to /sim/imu.

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

### Downloading Remaining Files
You'll be using files in `robosub-ros/simulation/personal`.

Make sure you have some version of Python3 installed on your personal computer. You can download and install it at [this link](https://www.python.org/downloads/release/python-381/).

## Running the Simulation
### Docker Simulation Setup
1. In the onboard container, make sure `roscore` is running.
2. In the landside container, run `cd dev/robosub-ros/simulation` and then `./runSim.sh &` (you may first need to run `chmod +x runSim.sh`). Wait until the terminal says `Initialization successful.` If it delays on the video compression library or meshcalc for an extended period of time, press enter a couple of times. This may be nothing more than confirmation-bias superstition.
3. Run whatever ROS topic publishing code you have. In `robosub-ros/simulation/docker`, there is a Python script `squareCommand.py` that you can run to make the robot move approximately in a square. (In reality, the robot will spin in wide circles because the robot isn't balanced.) You can use this script to test if communication between the simulations is working.

### Personal Computer Simulation Setup
1. Open CoppeliaSim.
2. Go to `File>Open Scene...` and open `empty_scene.ttt` in `robosub-ros/simulation/personal/scenes`.
3. Press the play button to start the simulation. The robot should start bobbing up and down.
4. In `robosub-ros/simulation/personal` on your local machine, run `python3 run_coppelia_comm.py`.
5. If the robot starts moving laterally, it worked!

#### A Note for Windows
On Windows machines with high res displays, some apps will not scale the font correctly. This happens because some applications essentially “tell” Windows that it will handle the scaling, but then don’t. To fix this, right click on the application icon, go to properties, compatibility, change high DPI settings, check override, and select system.

## Stopping/Shutting down the Simulation
To pause the simulation, just press the pause button to pause, and press play to resume.

To reset the simulation:
1. Stop ros_coppelia_comm.py
2. Stop the buoyancy simulation on your personal computer. 
3. Nothing needs to be done with the docker simulation.

To shut down the simulation completely:
1. Stop ros_coppelia_comm.py
2. Stop the buoyancy simulation on your personal computer.
3. Open a terminal in the docker container and run `ps -a`.
4. See which process ids correspond to CoppeliaSim and CoppeliaSim.sh, and run `kill -9 [processid]` for both. The `-9` argument tells the command to send a SIGKILL signal, rather than a SIGTERM. If you do not SIGKILL it, the processes appear to linger, which appears to interfere with communication between the simulations.
    
    It is currently unknown if this is fact or simply paranoia and superstition.
