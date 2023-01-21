# Task Planning

## Overview
### High Level Overview
Task planning is the control center - the brains - of the robot. It operates at the highest level of the software subsystems and controls the overall actions of the robot: what tasks the robot attempts and in what order, where the robot moves, when the robot moves to the next task, etc. It uses data from sensor fusion and computer vision and gives instructions to controls. See the [flow diagram](../../../../README.md#flow) in main README to see where it fits in the overall system.

### Code Structure
The robot is controlled through running a series of tasks arranged in a hierarchical state machine. We use the SMACH library to create and manage these state machines. Every task is a `smach.State` or `smach.StateMachine` object. A `smach.State` object's `execute()` function runs when the state machine transitions to that state. `execute()` returns a string that the state machine uses to determine which state to transition to next. [Documentation](http://wiki.ros.org/smach/Documentation) and [Tutorials](http://wiki.ros.org/smach/Tutorials) for SMACH can be found on the ROS wiki.

Take a look at this example and its output below.
```python
class ExampleTask(smach.State):
        
    def __init__(self):
        super().__init__(outcomes=["continue", "done"])
        print("This is an echo chamber:")
        self.count = 0
        
    def execute(self, userdata):
        print("echo")
        self.count += 1
        if self.count == 5:
            print("... you get the point")
            return "done"
        return "continue"

sm = smach.StateMachine(outcomes=['sm_done'])

with sm:
    smach.StateMachine.add("Example", ExampleTask(),
                           transitions={
                               'continue': 'Example',
                               'done': 'sm_done'
                           })

sm.execute()
```
The result of running the above code (ignoring SMACH's abundant logging) would be:
```
This is an echo chamber:
echo
echo
echo
echo
echo
... you get the point
```

### Design Decisions
- Non-blocking tasks:\
Every 


### Prioritization of Tasks

### Rationale (why did we design it this way)


## Running Code

### In Simulation

### On the Robot

## Guidelines for Writing Task Planning Code
