# CoppeliaSim Simulation for the AUV

These are the instructions to get CoppeliaSim running on your own personal computer, connected to the ROS Topics inside a Docker container. The simulation currently reads a [Float32MultiArray](http://docs.ros.org/melodic/api/std_msgs/html/msg/Float32MultiArray.html) of thruster values from the topic /sim/move. This is expected to be an array of 8 floats, each from -1 to 1. This array should be in the order top-front-right, top-front-left, top-back-right, top-back-left, bottom-front-right, bottom-front-left, bottom-back-right, bottom-back-left.

Additionally, it publishes a [TwistStamped](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/TwistStamped.html) to /sim/dvl, a [PoseStamped](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PoseStamped.html) to /sim/pose, and an [Imu](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Imu.html) to /sim/imu.

## Installing the Simulation

First, you will need a Docker container with port 8080 forwarded. The command currently in the [documentation repo](https://github.com/DukeRobotics/documentation/tree/master/docker) is:

`docker run -td -p 2200:2200 dukerobotics/robosub-ros`

You will need the command:

`docker run -td -p 2200:2200 -p 8080:8080 dukerobotics/robosub-ros`

which forwards port 8080. If you want the docker container mounted, run the command you normally do with `-p 8080:8080` added.

Once you have that, open your docker container in a terminal window and navigate to where you want your simulation stuff in a terminal window, and run the following commands (you can just copy paste the entire block into terminal).

```
source /opt/ros/kinetic/setup.bash
mkdir simulation
cd simulation
wget https://raw.githubusercontent.com/DukeRobotics/robosub-ros/simulation/src/simulation/production/docker/squareCommand.py
wget coppeliarobotics.com/files/CoppeliaSim_Edu_V4_0_0_Ubuntu16_04.tar.xz
tar -kxvf CoppeliaSim_Edu_V4_0_0_Ubuntu16_04.tar.xz
cd CoppeliaSim_Edu_V4_0_0_Ubuntu16_04
wget https://github.com/DukeRobotics/robosub-ros/raw/simulation/src/simulation/production/docker/servertest1.ttt
wget https://github.com/DukeRobotics/robosub-ros/raw/simulation/src/simulation/production/docker/libsimExtROSInterface.so
ls
```

Then run the following commands, one at a time.

```
sudo apt-get update
sudo apt-get install ros-kinetic-geometry2
sudo apt-get install xsltproc
```

### Downloading CoppeliaSim for Windows and Linux

On your personal computer, install the appropriate version of CoppeliaSim Edu for your system from the link [here](http://coppeliarobotics.com/downloads). 

### Downloading on Mac
1. Download the mac version of CoppeliaSim Edu
2. Unzip the download and move the resulting folder to Applications
3. To run, click on coppeliaSim inside of that folder
4. If you get an error about models not found, run the following at the top of the CoppeliaSim folder
```bash
sudo xattr -r -d com.apple.quarantine *
```

### Downloading Remaining Files

Additionally, download everything in the personal folder in this repo to your personal computer, inside your CoppeliaSim folder. For Windows, this will be at `C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu`.

### Important Note if you have Docker Toolbox

This only applies if you have a Windows machine, and it is not running Windows 10 Pro, Education, or Enterprise. You should open rosvrepcomm.py, located in the personal folder and comment out line 28, which reads:

```
clientID=vrep.simxStart('127.0.0.1',8080,True,True,5000,5)
```

And uncomment line 29, which reads:

```
#clientID=vrep.simxStart('192.168.99.100',8080,True,True,5000,5)
```

The reason for this is that Docker Toolbox handles the containers, and more specifically, their IP addresses, differently, requiring a different IP address. `192.168.99.100` is the default output of `docker-machine ip`, which is why it is used. If the simulation fails to connect, try running `docker-machine ip` to see if the output matches.


## Running the Simulation

In the docker container, navigate to the folder where you ran the commands above, then go to simulation/CoppeliaSim_Edu_V4_0_0_Ubuntu16_04/ and run, one at a time:

```
source /opt/ros/kinetic/setup.bash
roscore &
./coppeliaSim.sh -h -s servertest1.ttt
```

Wait until the terminal says `Initialization successful.` If it delays on the video compression library or meshcalc for an extended period of time, press enter a couple of times. This may be nothing more than confirmation-bias superstition.

In a new terminal window in the docker container, run whatever ROS topic publishing code you have. In the simulation folder in the docker container, there is a python script `squareCommand.py` that you can run to make the robot move approximately in a square. (In reality, the robot will spin in wide circles because the robot isn't balanced.)

Then, on your personal computer, open CoppeliaSim, open buoyancytestwithrob.tt, and run the simulation. The robot should bob up and down in the water. Then, run rosvrepcomm.py. If on mac, you must run rosvrepcomm.py with Python3, or else you'll run into library permission issues. The robot should start moving.

## Some very important notes!

To pause the simulation, just press the pause button. To reset the simulation, **first stop rosvrepcomm.py** then stop the buoyancy simulation. Nothing needs to be done with the docker simulation.

To shut down the simulation completely, follow the above steps, then open a new terminal in docker, run `ps -a`, see which process ids correspond to CoppeliaSim and CoppeliaSim.sh, and run `kill -9 [processid]` for both. If you do not SIGKILL it, the processes appear to linger, which appears to interfere with communication between the simulations.

It is currently unknown if this is fact or simply paranoia and superstition.
