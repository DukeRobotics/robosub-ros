# The best readme

docker run --priveleged --net=host -td -p 2201:2201 -v ${PWD}:/root/dev/robosub-ros dukerobotics/robosub-ros:landside

hostname

rostopic pub -r 10 /offboard/thruster_speeds custom_msgs/ThrusterSpeeds '{speeds: [0,0,0,0,0,0,0,0]}'

ping dukerobotics-laptop

roslaunch execute motion.launch
rosservice call /enable_controls true
rosrun controls test_state_publisher.py

hostname inside docker container on robot