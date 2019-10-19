#!/usr/bin/env python

from taskbase import TaskBase
import rospy


class WaitTask(TaskBase):

    DURATION = 10

    def __init__(self):
        super(WaitTask, self).__init__('wait')

    def run(self):
        if rospy.Time.now() - self.time_start > rospy.Duration(self.DURATION):
            return self.FINISHED
        return self.CONTINUE
