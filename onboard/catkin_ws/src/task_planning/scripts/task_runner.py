#!/usr/bin/env python3

import rospy
import smach.StateMachine
from gate_task import create_gate_task_sm


class TaskRunner:
    RATE = 30  # Hz

    def __init__(self):
        rospy.init_node("task_planning")

    def start(self):
        sm_top = smach.StateMachine(outcomes=['task_runner_succeeded', 'task_runner_failed'])

        with sm_top:
            sm_gate = create_gate_task_sm()
            smach.StateMachine.add('GATE_STATE', sm_gate,
                                   transitions={
                                       'gate_task_succeeded': 'task_runner_succeeded',
                                       'gate_task_failed': 'task_runner_failed'})


def main():
    try:
        TaskRunner().start()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
