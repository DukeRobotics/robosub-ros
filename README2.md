# The best readme

## Docker onboard
```bash
docker run -td --privileged --net=host -v /home/robot/robosub-ros:/root/dev/robosub-ros -v /dev:/dev dukerobotics/robosub-ros:onboard
```

## To launch the Docker container on landside container on the laptop during pool test
```bash
docker run --privileged --net=host -td -p 2201:2201 -v ${PWD}:/root/dev/robosub-ros dukerobotics/robosub-ros:landside
```

## To connect Robot onboard to laptop landside 

### On laptop landside
```bash
hostname
```

### On robot onboard
```bash
nano etc/hosts
```

```bash
ping {hostname}
```

## To test thrusters and onboard-landside connection
``roslaunch offboard_comms serial.launch``

``rostopic pub -r 10 /offboard/thruster_speeds custom_msgs/ThrusterSpeeds '{speeds: [0,0,0,0,0,0,0,0]}'``

This must be done with **serial.launch** instead of `motion.launch` because `motion.launch` launches 
`test_state_publisher.py` which also publishes to `offboard/thruster_speeds`.

## To set a desired global state for the robot
``roslaunch execute motion.launch``

``rosservice call /enable_controls true``

``rosrun controls test_state_publisher.py``

## To launch the joystick:
Start the raw joystick node on the **landside** computer by executing

`
roslaunch joystick_raw.launch
`

Start the joystick pulishing node on either computer by executing 

`
roslaunch pub_joy.launch
`
