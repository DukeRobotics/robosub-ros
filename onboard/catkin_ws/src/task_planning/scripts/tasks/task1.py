#!/usr/bin/env python

from taskbase import TaskBase
import rospy
from controls.msg import MoveWithSpeeds


class Task1(TaskBase):

    def __init__(self):
        self.controls_pub = rospy.Publisher('/controls/move', MoveWithSpeeds, queue_size=10)
        super(Task1, self).__init__('task1')

    def run(self):
        if rospy.Time.now() - self.time_start < rospy.Duration(10):
            return self.CONTINUE

        if rospy.Time.now() - (self.time_start + rospy.Duration(10)) < rospy.Duration(2):
            self.controls_pub.publish(MoveWithSpeeds([0.0, 0.0, -0.3, 0.0, 0.0, 0.0]))
            return self.CONTINUE

        if rospy.Time.now() - (self.time_start + rospy.Duration(12)) < rospy.Duration(50):
            self.controls_pub.publish(MoveWithSpeeds([0.2, 0.0, 0.0, 0.0, 0.0, 0.0]))
            return self.CONTINUE

        self.controls_pub.publish(MoveWithSpeeds([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
        return self.FINISHED
