# CoppeliaSim Simulation for the AUV
These are the instructions to get CoppeliaSim running on your own personal computer, connected to the ROS Topics inside a Docker container. The simulation currently reads a [Float32MultiArray](http://docs.ros.org/melodic/api/std_msgs/html/msg/Float32MultiArray.html) of thruster values from the topic /sim/move. This is expected to be an array of 8 floats, each from -1 to 1. This array should be in the order top-front-right, top-front-left, top-back-right, top-back-left, bottom-front-right, bottom-front-left, bottom-back-right, bottom-back-left.

Additionally, it publishes a [TwistStamped](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/TwistStamped.html) to /sim/dvl, a [PoseStamped](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PoseStamped.html) to /sim/pose, and an [Imu](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Imu.html) to /sim/imu.

## Installing the Simulation
### Setting up the Docker Container
1. You will need a Docker container with port 8080 forwarded. Run the command that you normally do to create your Docker container according to the [documentation repo](https://github.com/DukeRobotics/documentation/tree/master/docker), but with `-p 8080:8080` added.
    
    For example, you might run the command:

    `docker run -td -p 2200:2200 -p 8080:8080 dukerobotics/robosub-ros`
    
    [Here's](https://github.com/DukeRobotics/documentation/tree/master/docker) the link to the documentation repo with the Docker commands, if you need to look yours up.
    
    If you aren't mounting your files, and you don't have a particular care for where the files get located, you can just ssh into your Docker container and copy/paste the following giant block of commands into your terminal. This will take you to step 10 (running various `sudo apt-get` commands. Alternatively, you can follow all the steps.
    
    <details>
        <summary>Giant block of commands (click me)</summary><p>      
    
        
        git clone https://github.com/DukeRobotics/robosub-ros.git
        cd robosub-ros
        git checkout -b simulation origin/simulation
        cd ..
        wget coppeliarobotics.com/files/CoppeliaSim_Edu_V4_0_0_Ubuntu16_04.tar.xz
        tar -kxvf CoppeliaSim_Edu_V4_0_0_Ubuntu16_04.tar.xz
        cp robosub-ros/src/simulation/docker/libsimExtROSInterface.so CoppeliaSim_Edu_V4_0_0_Ubuntu16_04
        cp robosub-ros/src/simulation/docker/servertest1.ttt CoppeliaSim_Edu_V4_0_0_Ubuntu16_04
        ls
         
    </p></details>
        
2. Once you have a Docker container, ssh into your Docker container in a terminal window and navigate to where you want your simulation stuff in a terminal window. If mounted, navigate to where your files are mounted.
3. Run `git clone https://github.com/DukeRobotics/robosub-ros.git`, **unless you've mounted your files and already have the repo cloned on your personal computer**.
4. Navigate inside the repo with `cd robosub-ros`.
5. Checkout the simulation branch with `git checkout -b simulation origin/simulation`.
6. Navigate back to the directory above the git repo (i.e. the location from which you can call `cd robosub-ros`).
7. Run `wget coppeliarobotics.com/files/CoppeliaSim_Edu_V4_0_0_Ubuntu16_04.tar.xz` to download the CoppelliaSim installer.
8. Run `tar -kxvf CoppeliaSim_Edu_V4_0_0_Ubuntu16_04.tar.xz` to expand the files.
9. Run the following commands to copy important files into the CoppeliaSim folder.
    ```bash
    cp robosub-ros/src/simulation/docker/libsimExtROSInterface.so CoppeliaSim_Edu_V4_0_0_Ubuntu16_04
    cp robosub-ros/src/simulation/docker/servertest1.ttt CoppeliaSim_Edu_V4_0_0_Ubuntu16_04
    ```
10. Run each of the following commands, to install important files in the container. If you did all the above in your mounted files, this is the step you can skip to when making a new docker container.
    ```bash
    sudo apt-get update
    sudo apt-get install ros-kinetic-geometry2
    sudo apt-get install xsltproc
    ```

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
You can skip to step 3 if you're mounting your files, and you have access to the simulation branch files from your personal computer.

1. **If you don't already have the git repo on your personal computer**, run `git clone https://github.com/DukeRobotics/robosub-ros.git` to clone it to your computer.
2. Navigate into the repo in your git terminal and run `git checkout -b simulation origin/simulation`.

    The files that you'll be using are in `robosub-ros/src/simulation/personal`.
3. Make sure you have some version of python installed on your personal computer. **If you have a Mac, you must have Python 3!** You can download and install it at [this link](https://www.python.org/downloads/release/python-381/).

    You will need to run a python script on your personal computer (either from command line or from IDE) to have the simulations communicate with each other. For Macs, this script uses a library, which can cause permission errors when run by restricted programs. Because Python 2 is deprecated, it is a restricted program, while Python 3 is not.

## Running the Simulation
### Docker Simulation Setup
1. Open up a terminal and ssh into your Docker Container with the `-XY` flag (i.e. run `ssh -XY -p 2200 duke@[ip address]`).
2. Navigate into your CoppeliaSim folder. If you just copy/pasted the giant block of commands, run `cd CoppeliaSim_Edu_V4_0_0_Ubuntu16_04`.
3. Run `source /opt/ros/kinetic/setup.bash` and `roscore &` to be able to use ROS.
4. Run `./coppeliaSim.sh -h -s servertest1.ttt` to start the simulation in headless mode (with no graphics).

    Wait until the terminal says `Initialization successful.` If it delays on the video compression library or meshcalc for an extended period of time, press enter a couple of times. This may be nothing more than confirmation-bias superstition.
5. In a new terminal window, ssh into the Docker container and run whatever ROS topic publishing code you have. In `robosub-ros/src/simulation/docker`, there is a python script `squareCommand.py` that you can run to make the robot move approximately in a square. (In reality, the robot will spin in wide circles because the robot isn't balanced.) You can use this script to test if communication between the simulations is working.

### Personal Computer Simulation Setup
1. Open CoppeliaSim.
2. Go to `File>Open Scene...` and open `buoyancytestwithrob.ttt` in `robosub-ros/src/simulation/personal`.
3. Press the play button to start the simulation. The robot should start bobbing up and down.
4. On your computer, run `rosvrepcomm.py` at `robosub-ros/src/simulation/personal`. **If you have a Mac,** you must run this file with Python 3. **If you have Docker Toolbox (i.e. you have Windows, but not Windows 10 Pro, Education, or Enterprise)**, you need to run `rosvrepcomm.py` with the command line argument of the IP address of the Docker container (e.g. `rosvrepcomm.py 192.168.99.100`). By default this is `192.168.99.100`, but you can find the IP address by running `docker-machine ip` in your Docker Toolbox terminal.
    <details>
        <summary>Explanation (click me)</summary>        
        The reason for this is that Docker Toolbox handles the containers, and more specifically, their IP addresses, differently, requiring a different IP address. `192.168.99.100` is the default output of `docker-machine ip`, which is why it is used. If the simulation fails to connect, run `docker-machine ip` to see if the output is the IP address above.        
    </details>

5. If the robot starts moving laterally, it worked!

#### A Note for Windows
On windows machines with high res displays, some apps will not scale the font correctly. This happens because some applications essentially “tell” windows that it will handle the scaling, but then don’t. To fix this, right click on the application icon, go to properties, compatibility, change high DPI settings, check override, and select system.

## Stopping/Shutting down the Simulation
To pause the simulation, just press the pause button to pause, and press play to resume.

To reset the simulation:
1. Stop rosvrepcomm.py
2. Stop the buoyancy simulation on your personal computer. 
3. Nothing needs to be done with the docker simulation.

To shut down the simulation completely:
1. Stop rosvrepcomm.py
2. Stop the buoyancy simulation on your personal computer.
3. Open a terminal in the docker container and run `ps -a`.
4. See which process ids correspond to CoppeliaSim and CoppeliaSim.sh, and run `kill -9 [processid]` for both. If you do not SIGKILL it, the processes appear to linger, which appears to interfere with communication between the simulations.
    
    It is currently unknown if this is fact or simply paranoia and superstition.
