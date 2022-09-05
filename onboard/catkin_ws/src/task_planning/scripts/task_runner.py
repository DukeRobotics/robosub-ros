#!/usr/bin/env python3

import rospy
import smach
from move_tasks import MoveToPoseLocalTask
from tf import TransformListener


def main():
    rospy.init_node("task_runner")

    listener = TransformListener()

    sm_top = smach.StateMachine(outcomes=['finish'])
    with sm_top:
        smach.StateMachine.add('MoveForward2', MoveToPoseLocalTask(2, 0, -1, 0, 0, 0, listener),
                               transitions={'done': 'MoveLeft3'})
        smach.StateMachine.add('MoveLeft3', MoveToPoseLocalTask(0, -3, 0, 0, 0, 0, listener),
                               transitions={'done': 'MoveBack4'})
        smach.StateMachine.add('MoveBack4', MoveToPoseLocalTask(-4, 0, 0, 0, 0, 0, listener),
                               transitions={'done': 'finish'})
    sm_top.execute()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
