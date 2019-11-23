REMEMBER TO RE-COPY THE SCENE FILES

Install (one time only)
Container info

1. git clone (docker stuff)
2. tar -kxvf V-REP.tar.xz
3. export VREP_ROOT="./V-REP_PRO_EDU_V3_6_2_Ubuntu16_04/"
4. cd vrep_ros_plugins
sudo apt-get update
sudo apt-get install ros-kinetic-geometry2
sudo apt-get xsltproc
catkin config --extend /opt/ros/kinetic
5. catkin init
6. catkin build
7. source devel/setup.bash
8. cp -iv devel/lib/libv_repExtRosInterface.so "$VREP_ROOT/"


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