#!/usr/bin/env python

from taskbase import TaskBase
import rospy

class Task1(TaskBase):
    
    def __init__(self):
        super(Task1, self).__init__('task1')
    
    def run(self):
        rospy.loginfo('TASK 1')
        return self.FINISHED
