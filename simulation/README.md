# CoppeliaSim Simulation for the AUV
These are the instructions to get CoppeliaSim running on your own personal computer, connected to the ROS Topics inside a Docker container. The simulation currently reads a [Float32MultiArray](http://docs.ros.org/melodic/api/std_msgs/html/msg/Float32MultiArray.html) of thruster values from the topic /sim/move. This is expected to be an array of 8 floats, each from -1 to 1. This array should be in the order top-front-right, top-front-left, top-back-right, top-back-left, bottom-front-right, bottom-front-left, bottom-back-right, bottom-back-left.

Additionally, it publishes a [TwistStamped](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/TwistStamped.html) to /sim/dvl, a [PoseStamped](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PoseStamped.html) to /sim/pose, and an [Imu](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Imu.html) to /sim/imu.

## Installing the Simulation
### Assumptions
This readme assumes that you are able to mount the git repo in the docker container. 

### Setting up the Docker Container
You will need a Docker container with port 8080 forwarded. Run the command that you normally do to create your Docker container according to the [documentation repo](https://github.com/DukeRobotics/documentation/tree/master/docker), but with `-p 8080:8080` added.
    
For example, you might run the command:

`docker run -td -p 2200:2200 -p 8080:8080 --mount type=bind,source=C:\Users\Eric\Documents\Robotics\CS,target=/home/duke/dev/robosub-ros/src dukerobotics/robosub-ros`
    
[Here's](https://github.com/DukeRobotics/documentation/tree/master/docker) the link to the documentation repo with the Docker commands, if you need to look yours up.

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

    You will need to run a python script on your personal computer (either from command line or from IDE) to have the simulations communicate with each other. For Macs, this script uses a library, which can cause permission errors when run by restricted programs. Because Python 2 is deprecated, it is a restricted program, while Python 3 is not.

## Running the Simulation
### Docker Simulation Setup
1. Open up a terminal and ssh into your Docker Container with the `-XY` flag (i.e. run `ssh -XY -p 2200 duke@[ip address]`).
2. Run `source /opt/ros/kinetic/setup.bash` and `roscore &`.
3. Run `~/dev/runSim.sh &`. Wait until the terminal says `Initialization successful.` If it delays on the video compression library or meshcalc for an extended period of time, press enter a couple of times. This may be nothing more than confirmation-bias superstition.
4. Run whatever ROS topic publishing code you have. In `robosub-ros/simulation/docker`, there is a python script `squareCommand.py` that you can run to make the robot move approximately in a square. (In reality, the robot will spin in wide circles because the robot isn't balanced.) You can use this script to test if communication between the simulations is working.

### Personal Computer Simulation Setup
1. Open CoppeliaSim.
2. Go to `File>Open Scene...` and open `empty_scene.ttt` in `robosub-ros/simulation/personal/scenes`.
3. Press the play button to start the simulation. The robot should start bobbing up and down.
4. On your computer, run `ros_coppelia_comm.py` at `robosub-ros/src/simulation/personal`. **If you have a Mac,** you must run this file with Python 3. **If you have Docker Toolbox (i.e. you have Windows, but not Windows 10 Pro, Education, or Enterprise)**, you need to run `ros_coppelia_comm.py` with the command line argument of the IP address of the Docker container (e.g. `ros_coppelia_comm.py 192.168.99.100`). By default this is `192.168.99.100`, but you can find the IP address by running `docker-machine ip` in your Docker Toolbox terminal.
    <details>
        <summary>Click for Explanation</summary>        
        The reason for this is that Docker Toolbox handles the containers, and more specifically, their IP addresses, differently, requiring a different IP address. `192.168.99.100` is the default output of `docker-machine ip`, which is why it is used. If the simulation fails to connect, run `docker-machine ip` to see if the output is the IP address above.        
    </details>

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
4. See which process ids correspond to CoppeliaSim and CoppeliaSim.sh, and run `kill -9 [processid]` for both. If you do not SIGKILL it, the processes appear to linger, which appears to interfere with communication between the simulations.
    
    It is currently unknown if this is fact or simply paranoia and superstition.
