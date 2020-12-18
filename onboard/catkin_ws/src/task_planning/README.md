# Task Planning

## Overview
### High Level Overview
Task planning is like the control center of the robot. It uses data from sensor fusion and vision and gives instructions to controls. It operates at the highest level of the software subsystems and controls the overall actions of the robot: what tasks the robot attempts and in what order, where the tries to move, robot moves, when the robot moves to the next task, etc. See the following flow diagram (from main README):
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

### Code Structure
The task planning code is designed in a hierarchy built on the `task` object (i.e. higher level tasks inherit from the `task` object). The structure operates similarly to Arduino (which has `setup()` and `loop()` methods). The `task` object has an `_on_task_start` method: analogous to `setup()` in Arduino, this is run *once* the first time the task is run (i.e. when `task.run()` is called). This differs from the `init` method which runs once when the task is initialized. The `task` object also has an `_on_task_run` method: analogous to `loop()` in Arduino, this method is run over and over. For example, see the follwoing code:
```python
class ExampleTask(Task):
        
    def _on_task_start(self):
        print("This is an echo chamber:")
        self.count = 0
        
    def _on_task_run(self):
        print("echo")
        self.count += 1
        if self.count == 5:
            print("... you get the point")
            self.finish()

ExampleTask().run()
```
The result of running the above code would be:
```
This is an echo chamber:
echo
echo
echo
echo
echo
... you get the point
```


### Prioritization of Tasks

### Rationale (why did we design it this way)


## Running Code

### In Simulation

### On the Robot

## Guidelines for Writing Task Planning Code
