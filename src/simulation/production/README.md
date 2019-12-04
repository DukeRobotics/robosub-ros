#ANY ROGUE COPPELIASIM PROCESSES MUST BE SIGKILLED OR ELSE EVERYTHING GOES TO HELL

REMEMBER TO RE-COPY THE SCENE FILES

Install (one time only)
Container info


source /opt/ros/kinetic/setup.bash
mkdir simulation
cd simulation
wget https://raw.githubusercontent.com/DukeRobotics/robosub-ros/simulation/src/simulation/production/docker/squareCommand.py
wget coppeliarobotics.com/files/CoppeliaSim_Edu_V4_0_0_Ubuntu16_04.tar.xz
tar -kxvf CoppeliaSim_Edu_V4_0_0_Ubuntu16_04.tar.xz
cd CoppeliaSim_Edu_V4_0_0_Ubuntu16_04
wget https://github.com/DukeRobotics/robosub-ros/raw/simulation/src/simulation/production/docker/servertest1.ttt
wget https://github.com/DukeRobotics/robosub-ros/raw/simulation/src/simulation/production/docker/libsimExtROSInterface.so

sudo apt-get update
sudo apt-get install ros-kinetic-geometry2
sudo apt-get install xsltproc

Necessary?
git clone --recursive https://github.com/CoppeliaRobotics/simExtROSInterface.git sim_ros_interface
cd sim_ros_interface
catkin config --extend /opt/ros/kinetic
5. catkin init
6. catkin build
7. source devel/setup.bash
8. ./install.sh
copy the thing (build/devel/lib?)

cd C	
ls



Personal computer
Download stuff


Testing/Running
1. In the container, start your publishing code. The bot currently publishes the 

source /opt/ros/kinetic/setup.bash
roscore &
./coppeliaSim.sh -h -s servertest1.ttt
(new window)
python squareCommand.py

mash enter if stuck on video compression lib or meshcalc (just sometimes for meshcalc)

List of assumptions:
mass evenly distributed (across bounding box or object?)
	uniform density?
thrusters have full power above water
thrusters 