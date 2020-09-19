# CoppeliaSim Simulation for the AUV
These are the instructions to get CoppeliaSim running on your own personal computer, connected to the ROS Topics inside a Docker container. The simulation currently reads a [Float32MultiArray](http://docs.ros.org/melodic/api/std_msgs/html/msg/Float32MultiArray.html) of thruster values from the topic /sim/move. This is expected to be an array of 8 floats, each from -1 to 1. This array should be in the order top-front-right, top-front-left, top-back-right, top-back-left, bottom-front-right, bottom-front-left, bottom-back-right, bottom-back-left.

Additionally, it publishes a [TwistStamped](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/TwistStamped.html) to /sim/dvl, a [PoseStamped](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PoseStamped.html) to /sim/pose, and an [Imu](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Imu.html) to /sim/imu.

## Installing the Simulation
### Assumptions
This readme assumes that you are able to mount the git repo in the docker container, and that you have the most recent Docker image pulled. 

### Downloading CoppeliaSim on your personal computer
#### Downloading on Windows
Download the installer for the Edu version from [this link](http://coppeliarobotics.com/winVersions), and run the downloaded .exe file.

#### Downloading on Linux
Download the file for the Edu version for your OS from [this link](http://coppeliarobotics.com/ubuntuVersions), move the tar file to where you'd like the CoppeliaSim folder, and run `tar -kxvf [filename]`.

#### Downloading on Mac
1. Download the mac version of CoppeliaSim Edu from [this link](http://coppeliarobotics.com/downloads)
2. Unzip the download and move the resulting folder to Applications
3. To run, click on coppeliaSim inside of that folder
4. If you get an error about models not found, run the following at the top of the CoppeliaSim folder
```bash
sudo xattr -r -d com.apple.quarantine *
```

### Downloading Remaining Files
You'll be using files in `robosub-ros/simulation/personal`.

Make sure you have some version of python installed on your personal computer. **If you have a Mac, you must have Python 3!** You can download and install it at [this link](https://www.python.org/downloads/release/python-381/).

<details>
    <summary>Click for Explanation of Python3</summary>
    You will need to run a python script on your personal computer (either from command line or from IDE) to have the simulations communicate with each other. For Macs, this script uses a library, which can cause permission errors when run by restricted programs. Because Python 2 is deprecated, it is a restricted program, while Python 3 is not.
</details>

## Running the Simulation
### Docker Simulation Setup
1. Open up a terminal and ssh into your Docker Container with the `-XY` flag (i.e. run `ssh -XY -p 2201 duke@localhost`).
2. Run `source /opt/ros/melodic/setup.bash` and `roscore &`.
3. Run `cd dev/robosub-ros/simulation` and then `./runSim.sh &` (you may first need to run `chmod +x runSim.sh`). Wait until the terminal says `Initialization successful.` If it delays on the video compression library or meshcalc for an extended period of time, press enter a couple of times. This may be nothing more than confirmation-bias superstition.
4. Run whatever ROS topic publishing code you have. In `robosub-ros/simulation/docker`, there is a python script `squareCommand.py` that you can run to make the robot move approximately in a square. (In reality, the robot will spin in wide circles because the robot isn't balanced.) You can use this script to test if communication between the simulations is working.

### Personal Computer Simulation Setup
1. Open CoppeliaSim.
2. Go to `File>Open Scene...` and open `empty_scene.ttt` in `robosub-ros/simulation/personal/scenes`.
3. Press the play button to start the simulation. The robot should start bobbing up and down.
4. On your computer, run `ros_coppelia_comm.py` at `robosub-ros/simulation/personal`. **If you have a Mac,** you must run this file with Python 3.
5. If the robot starts moving laterally, it worked!

#### A Note for Windows
On windows machines with high res displays, some apps will not scale the font correctly. This happens because some applications essentially “tell” windows that it will handle the scaling, but then don’t. To fix this, right click on the application icon, go to properties, compatibility, change high DPI settings, check override, and select system.

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
