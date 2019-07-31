#!/usr/bin/env python

from taskbase import TaskBase
import rospy
from controls.msg import MoveWithSpeeds

class QualBasic(TaskBase):


    def __init__(self):
        self.controls_pub = rospy.Publisher('/controls/move', MoveWithSpeeds, queue_size=10)
        super(QualBasic, self).__init__('qualbasic')

    def pre_run(self):
        self.time_start = rospy.Time.now()

    def run(self):
        if rospy.Time.now() - self.time_start < rospy.Duration(10):
            return self.CONTINUE

        if rospy.Time.now() - (self.time_start + rospy.Duration(10)) < rospy.Duration(2):
            self.controls_pub.publish(MoveWithSpeeds([0.2, 0.0, -0.2, 0.0, 0.0, 0.0]))
            return self.CONTINUE

        if rospy.Time.now() - (self.time_start + rospy.Duration(12)) < rospy.Duration(5):
            self.controls_pub.publish(MoveWithSpeeds([0.2, 0.0, 0.0, 0.0, 0.0, 0.0]))
            return self.CONTINUE

        return self.FINISHED