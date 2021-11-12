#!/usr/bin/env python

import rospy
import smach
import random
from task import Task
from move_tasks import MoveToPoseGlobalTask, MoveToPoseLocalTask, AllocateVelocityLocalTask
from time import sleep
from tf import TransformListener

# define state Foo

def main():
    rospy.init_node('smach_test')
    # listener = TransformListener()
    print("before sleep")
    sleep(2)
    # print("before ctor")
    move = AllocateVelocityLocalTask(0, 0, -1, 0, 0, 0)
    #move.run(None)
    #return
    rate = rospy.Rate(15)
    while True:
        move.run(None)
        rate.sleep()
    # move = MoveToPoseLocalTask(2, 0, 0, 0, 0, 0, listener)
    
    # move.run(None)
    # print("first move")
    # move = MoveToPoseGlobalTask(2, 4, 0, 0, 0, 0)
    # move.run(None)
    # print("AAAAAAAAAAAAAAAAAA")
    # return
    

    # t = AllocateVelocityGlobalTask(0.2, 0, 0, 0, 0, 0)
    # while(True):
    #     t.run()
    
    sm = controls_testing()

    # Execute SMACH plan
    outcome = sm.execute()

def controls_testing():
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['finish'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Move1', MoveToPoseGlobalTask(0, 2, 0, 0, 0, 0), 
                               transitions={'done':'Move1'})
        # left square
        smach.StateMachine.add('MoveLeft2', MoveToPoseGlobalTask(2, 2, 0, 0, 0, 0), 
                               transitions={'done':'MoveLeft3'})
        smach.StateMachine.add('MoveLeft3', MoveToPoseGlobalTask(0, 2, 0, 0, 0, 0), 
                               transitions={'done':'MoveLeft4'})
        smach.StateMachine.add('MoveLeft4', MoveToPoseGlobalTask(0, 0, 0, 0, 0, 0), 
                               transitions={'done':'finish'})
        # right square
        smach.StateMachine.add('MoveRight2', MoveToPoseGlobalTask(2, -2, 0, 0, 0, 0), 
                               transitions={'done':'MoveRight3'})
        smach.StateMachine.add('MoveRight3', MoveToPoseGlobalTask(0, -2, 0, 0, 0, 0), 
                               transitions={'done':'MoveRight4'})
        smach.StateMachine.add('MoveRight4', MoveToPoseGlobalTask(0, 0, 0, 0, 0, 0), 
                               transitions={'done':'finish'})

    return sm

def concurrency():
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['finish'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Move1', MoveToPoseGlobalTask(2, 0, 0, 0, 0, 0), 
                               transitions={'done':'ConcurrentMove2'})
        cc = smach.Concurrence(outcomes = ['done'],
                    default_outcome = 'done',
                    outcome_map = {'done':{'Move2':'done', # end concurrency when Move2 and Log have both completed
                                           'Log':'done'}})
        with cc:
            smach.Concurrence.add('Move2', MoveToPoseGlobalTask(2, 2, 0, 0, 0, 0))
            smach.Concurrence.add('Log', LogSomethingUseful())

        smach.StateMachine.add('ConcurrentMove2', cc, transitions={'done':'Move3'})
        
        smach.StateMachine.add('Move3', MoveToPoseGlobalTask(0, 2, 0, 0, 0, 0), 
                               transitions={'done':'Move4'})
        smach.StateMachine.add('Move4', MoveToPoseGlobalTask(0, 0, 0, 0, 0, 0), 
                               transitions={'done':'finish'})

    return sm

def decision_making():
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['finish'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Move1', MoveToPoseGlobalTask(2, 0, 0, 0, 0, 0), 
                               transitions={'done':'choice'})
        smach.StateMachine.add("choice", RandomChoice(2),
                                transitions={'0':'MoveLeft2', '1':'MoveRight2'})
        # left square
        smach.StateMachine.add('MoveLeft2', MoveToPoseGlobalTask(2, 2, 0, 0, 0, 0), 
                               transitions={'done':'MoveLeft3'})
        smach.StateMachine.add('MoveLeft3', MoveToPoseGlobalTask(0, 2, 0, 0, 0, 0), 
                               transitions={'done':'MoveLeft4'})
        smach.StateMachine.add('MoveLeft4', MoveToPoseGlobalTask(0, 0, 0, 0, 0, 0), 
                               transitions={'done':'finish'})
        # right square
        smach.StateMachine.add('MoveRight2', MoveToPoseGlobalTask(2, -2, 0, 0, 0, 0), 
                               transitions={'done':'MoveRight3'})
        smach.StateMachine.add('MoveRight3', MoveToPoseGlobalTask(0, -2, 0, 0, 0, 0), 
                               transitions={'done':'MoveRight4'})
        smach.StateMachine.add('MoveRight4', MoveToPoseGlobalTask(0, 0, 0, 0, 0, 0), 
                               transitions={'done':'finish'})

    return sm

class RandomChoice(Task):
    """Randomly chooses a number"""

    def __init__(self, options):
        super(RandomChoice, self).__init__(outcomes=[str(i) for i in range(options)])

        self.options = options

    def run(self, userdata):
        res = str(random.randint(0, self.options))
        rospy.loginfo(res)
        return res

class LogSomethingUseful(Task):
    """Logs Stuff"""

    def __init__(self):
        super(LogSomethingUseful, self).__init__(outcomes=["done"])

    def run(self, userdata):
        rate = rospy.Rate(1)
        for i in range(40):
            rospy.loginfo("something important")
            rate.sleep()
        return "done"


if __name__ == '__main__':
    main()