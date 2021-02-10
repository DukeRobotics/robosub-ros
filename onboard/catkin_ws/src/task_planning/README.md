# Task Planning

## Overview
### High Level Overview
Task planning is the control center - the brains - of the robot. It operates at the highest level of the software subsystems and controls the overall actions of the robot: what tasks the robot attempts and in what order, where the robot moves, when the robot moves to the next task, etc. It uses data from sensor fusion and computer vision and gives instructions to controls. See the [flow diagram](../../../../README.md#flow) in main README to see where it fits in the overall system.

### Code Structure
The robot is controlled through running a series of tasks. Every task inherits from the `Task` object. The `Task` object's  `_on_task_start()` function runs *once* when the task is run using `Task.run()`. Then `_on_task_run()` runs repeatedly until the task is finished using `Task.finish()`. If you have used [Arduino](https://www.arduino.cc/en/software) or [Processing](https://processing.org/) before, this is analagous to the `setup()` and `loop()` methods.

Take a look at this example and its output below.
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
