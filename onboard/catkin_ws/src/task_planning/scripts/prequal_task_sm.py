#!/usr/bin/env python

import smach
import rospy
from task import Task
from move_tasks import MoveToPoseLocalTask
from time import sleep


prequal_TICK_CV_NAME = "bin"  # change back to prequal_tick


def main():
    rospy.init_node('prequal_task')

    sm = create_prequal_task_sm()
    sleep(2)
    # Execute SMACH plan
    sm.execute()


def create_prequal_task_sm(velocity=0.2):
    sm = smach.StateMachine(outcomes=['prequal_task_succeeded', 'prequal_task_failed'])
    with sm:
        def concurrence_term_wait_loop_cb(outcome_map):
            return outcome_map['WAIT_DONE'] == 'done'

        def concurrence_term_dive_forward_loop_cb(outcome_map):
            return outcome_map['DIVE_FORWARD'] == 'done'

        def concurrence_term_move_forward_loop_cb(outcome_map):
            return outcome_map['MOVE_FORWARD'] == 'done'

        dive_forward_cc = smach.Concurrence(outcomes=['done'],
                                            default_outcome='done',
                                            child_termination_cb=concurrence_term_wait_loop_cb,
                                            outcome_map={'done': {'WAIT_DONE': 'done'}})
        with dive_forward_cc:
            smach.Concurrence.add('WAIT_DONE', WaitTask(5))
            smach.Concurrence.add('DIVE_FORWARD', MoveToPoseLocalTask(2, 0, -2, 0, 0, 0))

        move_foward_cc = smach.Concurrence(outcomes=['done'],
                                           default_outcome='done',
                                           child_termination_cb=concurrence_term_wait_loop_cb,
                                           outcome_map={'done': {'WAIT_DONE': 'done'}})
        with move_foward_cc:
            smach.Concurrence.add('WAIT_DONE', WaitTask(5))
            smach.Concurrence.add('MOVE_FORWARD', MoveToPoseLocalTask(5, 0, 0, 0, 0, 0))

        smach.StateMachine.add('DIVE_FORWARD', dive_forward_cc, transitions={'done': 'MOVE_FORWARD'})
        smach.StateMachine.add('MOVE_FORWARD', move_foward_cc, transitions={'done': 'prequal_task_succeeded'})

    return sm


class WaitTask(Task):
    def __init__(self, time):
        super(WaitTask, self).__init__(["done"])
        self.time = time

    def run(self, userdata):
        sleep(self.time)
        return "done"


if __name__ == '__main__':
    main()
