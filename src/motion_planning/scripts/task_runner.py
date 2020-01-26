#!/usr/bin/env python

import rospy
from task import Task
from template_tasks import *

class TaskRunner(object):
    RATE = 30 # Hz

    def __init__(self):
        from competition_task import CompetitionTask
        self.task = CompetitionTask
        self.rate = rospy.Rate(self.RATE)
        self.state = TaskState()
        

    def start(self):
        while not self.task.finished:
            self.task.run()
            self.rate.sleep()

rospy.init_node("task_planning")
TaskRunner().start()