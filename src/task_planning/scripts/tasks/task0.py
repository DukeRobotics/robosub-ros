#!/usr/bin/env python

from taskbase import TaskBase
import rospy

class Task0(TaskBase):

    def __init__(self):
        super(Task0, self).__init__('task0')

    def run(self):
        rospy.loginfo('yo yo')
        return self.CONTINUE
