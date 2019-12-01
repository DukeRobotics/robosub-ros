REMEMBER TO RE-COPY THE SCENE FILES

Install (one time only)
Container info

1. git clone (docker stuff)
2. tar -kxvf V-REP.tar.xz ----------------------------------------------wget
3. export VREP_ROOT="./V-REP_PRO_EDU_V3_6_2_Ubuntu16_04/"
	needs to be coppeliasim
4. cd vrep_ros_plugins -----------------------------------------------FIXME
sudo apt-get update
sudo apt-get install ros-kinetic-geometry2
sudo apt-get xsltproc
git clone --recursive https://github.com/CoppeliaRobotics/simExtROSInterface.git sim_ros_interface
cd sim_ros_interface
catkin config --extend /opt/ros/kinetic
5. catkin init
6. catkin build
7. source devel/setup.bash
source /opt/ros/kinetic/setup.bash
8. ./install.sh
copy the thing (build/devel/lib?)


Personal computer
1. git clone (personal stuff)
2. Install V-REP (file in folder)


Testing/Running
1. In the container, start your publishing code. The bot currently publishes the 

start ros pub code
source ros and run docker headless

mash enter if stuck on video compression lib or meshcalc (just sometimes for meshcalc)

List of assumptions:
mass evenly distributed (across bounding box or object?)
	uniform density?
thrusters have full power above water
thrusters 