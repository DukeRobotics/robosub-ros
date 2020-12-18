# Task Planning

## Overview
Task planning is like the control center of the robot. It operates at the highest level of the software subsystems and controls the overall actions of the robot: 
what tasks the robot attempts and in what order, where the tries to move, robot moves, when the robot moves to the next tasks, etc. Task planning takes data from sensor fusion 
and vision and outputs instructions to controls. See the following flow diagram (from main README)

```
        sensors (IMU, DVL, etc.)                cameras
            \                                     / \
             \                                   /   \
              v                                 /     v
            Data Pub                           /   Camera View
                \                             /
 Simulation ---> \                           /
                  v                         v
                Sensor Fusion          Computer Vision
                    \                     /
                     \                   /
                      v                 v
                         Task Planning
                               |
                               |
                               v
              Joystick ---> Controls
                               |
                               | ---> Simulation
                               v
                         Offboard Comms
                               |
                               |
                               v
                    thrusters, actuators, etc.

```


## Running Code

### In Simulation

### On the Robot
