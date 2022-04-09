#!/usr/bin/env python

from combination_tasks import ListTask
from move_tasks import MoveToPoseGlobalTask
from move_tasks import MoveToPoseLocalTask
from task import Task
import smach
import rospy
import gate_task
from tf import TransformListener
import math
import time

def main():
    rospy.init_node('gate_task')
    
    sm = create_prequal_task_sm()
    time.sleep(2)
    # Execute SMACH plan
    outcome = sm.execute()

def create_prequal_task_sm():
    sm = smach.StateMachine(outcomes=['prequal_task_succeeded', 'prequal_task_failed'])
    listener = TransformListener()
    gate_sm = gate_task.create_gate_task_sm()

    with sm:
        smach.StateMachine.add('GO_THROUGH_GATE', gate_sm,
                               transitions={
                                   'gate_task_succeeded': 'APPROACH_MARKER',
                                   'gate_task_failed': 'prequal_task_failed'})
        smach.StateMachine.add('APPROACH_MARKER', MoveToPoseLocalTask(2, 0, 0, 0, 0, 0, listener),
                               transitions={
                                   'done': 'LEFT_OF_MARKER'})
        smach.StateMachine.add('LEFT_OF_MARKER', MoveToPoseLocalTask(1, 1, 0, 0, 0, 0, listener), # TODO fill in args
                               transitions={
                                   'done': 'BEHIND_MARKER'})
        smach.StateMachine.add('BEHIND_MARKER', MoveToPoseLocalTask(1, -1, 0, 0, 0, 0, listener),
                               transitions={
                                   'done': 'RIGHT_OF_MARKER'})
        smach.StateMachine.add('RIGHT_OF_MARKER', MoveToPoseLocalTask(-1, -1, 0, 0, 0, 0, listener),
                               transitions={
                                   'done': 'BACK_THROUGH_GATE'})
        smach.StateMachine.add('BACK_TO_FRONT_OF_MARKER', MoveToPoseLocalTask(-1, 1, 0, 0, 0, math.PI, listener),
                               transitions={
                                   'done': 'BACK_THROUGH_GATE'})
        smach.StateMachine.add('BACK_THROUGH_GATE', gate_sm,
                               transitions={
                                   'gate_task_succeeded': 'prequal_task_succeeded',
                                   'gate_task_failed': 'prequal_task_failed'})

    return sm


if __name__ == '__main__':
    main()




# class PreQualGlobalTask(Task):
#     """
#     Task to complete prequalifying run by moving to global poses
#     """

#     POSE1 = [0, 0, 0, 0, 0, 0]
#     POSE2 = [0, 0, 0, 0, 0, 0]
#     POSE3 = [0, 0, 0, 0, 0, 0]
#     POSE4 = [0, 0, 0, 0, 0, 0]
#     POSE5 = [0, 0, 0, 0, 0, 0]

#     def __init__(self):
#         super(PreQualGlobalTask, self).__init__()

#         self.list_task = ListTask([MoveToPoseGlobalTask(*self.POSE1),
#                                    MoveToPoseGlobalTask(*self.POSE2),
#                                    MoveToPoseGlobalTask(*self.POSE3),
#                                    MoveToPoseGlobalTask(*self.POSE4),
#                                    MoveToPoseGlobalTask(*self.POSE5)])

#     def _on_task_run(self):
#         self.list_task.run()

#         if self.list_task.finished:
#             self.finish()


# class PreQualLocalTask(Task):
#     """
#     Task to complete prequalifying run by moving to local poses
#     """

#     POSE1 = [0, 0, 0, 0, 0, 0]
#     POSE2 = [0, 0, 0, 0, 0, 0]
#     POSE3 = [0, 0, 0, 0, 0, 0]
#     POSE4 = [0, 0, 0, 0, 0, 0]
#     POSE5 = [0, 0, 0, 0, 0, 0]

#     def __init__(self):
#         super(PreQualLocalTask, self).__init__()

#         self.list_task = ListTask([MoveToPoseLocalTask(*self.POSE1),
#                                    MoveToPoseLocalTask(*self.POSE2),
#                                    MoveToPoseLocalTask(*self.POSE3),
#                                    MoveToPoseLocalTask(*self.POSE4),
#                                    MoveToPoseLocalTask(*self.POSE5)])

#     def _on_task_run(self):
#         self.list_task.run()

#         if self.list_task.finished:
#             self.finish()
