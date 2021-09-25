#!/usr/bin/env python

import rospy
import smach
from move_tasks import MoveToPoseGlobalTask

# define state Foo

# main
def main():
    rospy.init_node('smach_test')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['finish'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Move1', MoveToPoseGlobalTask(2, 0, 0, 0, 0, 0), 
                               transitions={'spin':'Move1',
                                            'done':'Move2'})
        smach.StateMachine.add('Move2', MoveToPoseGlobalTask(2, 2, 0, 0, 0, 0), 
                               transitions={'spin':'Move2',
                                            'done':'Move3'})
        smach.StateMachine.add('Move3', MoveToPoseGlobalTask(0, 2, 0, 0, 0, 0), 
                               transitions={'spin':'Move3',
                                            'done':'Move4'})
        smach.StateMachine.add('Move4', MoveToPoseGlobalTask(0, 0, 0, 0, 0, 0), 
                               transitions={'spin':'Move4',
                                            'done':'finish'})

    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()