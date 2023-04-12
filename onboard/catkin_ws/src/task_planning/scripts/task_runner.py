#!/usr/bin/env python3

import rospy
import smach
import tf
from smach_test import controls_testing
from interface.controls import ControlsInterface
from buoy_task import BuoyTask
from interface.cv import CVInterface
from move_tasks import MoveToPoseGlobalTask
import random


class TaskRunner(smach.StateMachine):
    RATE = 30  # Hz

    def __init__(self):
        super(TaskRunner, self).__init__(outcomes=['done'])
        rospy.init_node("task_planning")
        self.listener = tf.TransformListener()
        self.controls = ControlsInterface(self.listener)
        
        self.x, self.y = random.randint(-10, 10), random.randint(-10, 10)

        with self:
            # smach.StateMachine.add('TEST', BuoyTask(self.listener, self.controls, CVInterface()),
            #                        transitions={'done': 'done'})
            smach.StateMachine.add('TEST', MoveToPoseGlobalTask(self.x, self.y, 0, 0, 0, 0, self.controls),
                                   transitions={'done': 'done', 'continue': 'TEST'})
            # smach.StateMachine.add('TEST', controls_testing(self.controls, self.listener),
            #                        transitions={'done': 'done'})

    def execute(self):
        rospy.loginfo("Waiting for transform listener")
        self.listener.waitForTransform('odom', 'base_link', rospy.Time(), rospy.Duration(15))
        super(TaskRunner, self).execute()


def main():
    try:
        TaskRunner().execute()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
