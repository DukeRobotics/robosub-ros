#!/usr/bin/env python3

import rospy
import smach
import tf
from smach_test import simple_move_test, controls_testing
from interface.controls import ControlsInterface


class TaskRunner(smach.StateMachine):
    RATE = 30  # Hz

    def __init__(self):
        super(TaskRunner, self).__init__(outcomes=['done'])
        rospy.init_node("task_planning")
        self.listener = tf.TransformListener()
        self.controls = ControlsInterface(self.listener)

        with self:
            smach.StateMachine.add('SIMPLE_MOVE_TEST', controls_testing(self.controls, self.listener),
                                   transitions={
                                       'finish': 'done'})

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
