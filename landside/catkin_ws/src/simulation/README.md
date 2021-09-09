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

### Object IDs
Each object inside the simulation has a unique ID used to identify its type in order to determine what 
CV data to send. The IDs are:
- 1: Gate
    - Size: 60 x 120 in.
- 2: Buoy
    - size: 48x24 in. piece of paper, oriented facing sideways
- 3: Bin
    - Size: 1/3 of the bin is covered by a 8x14x0.5 in. plate
- 4: Octagon
    - Size: 2.7m diameter
- 5: Torpedo
    - Size: 2x2x6 in.
- 6: Path marker
    - Size: 1.2x0.15x0.075 m
- 7: Gate side identifier (*Choose your side*)
    - Size: 6x12 in.

### How to add a new object type
1. Define the ID that the object will use (for the "Object IDs" section), and 
ensure that no other object is already using this ID.
2. In your favorite CAD software (I'm using OnShape), make a quick approximation
of the object and export it as an STL file.
3. In CoppeliaSim, go to the Menu Bar and click File->Import->Mesh... and select the STL file. Click
'Import' in the resulting dialog box wtihout changing any settings.
4. With the imported object selected in the object hierarchy view,
 go to the Menu Bar and select Tools->Scene Object Properties.
5. (If the scale of the shape is wrong): , and in the properties popup window click 'View/modify geometry' near the bottom. From there you can modify the size of the object.
6. Go to the 'Common' tab in the Properties window, and ensure that the check
box titled 'Object is model base' toward the bottom.
7. Double click on the name of the object in the object hierarchy to give
it the appropriate name.
8. In the Menu Bar go to File->Save model as... and follow the resulting prompt
to save the model as a .ttm model in the personal/models/ folder.

You can now import the model to any scene using File->Load model...!
