#!/usr/bin/env python

import rospy
from task import Task
from competition_task import CompetitionTask
from task_state import TaskState

class TaskRunner:

    RATE = 15 # Hz

    def __init__(self):
        rospy.init_node("task_planning")
        self.competition_task = CompetitionTask()

    def start(self):
        self.rate = rospy.Rate(self.RATE)

        while not self.competition_task.finished:
            self.competition_task.run()
            self.rate.sleep()


TaskRunner().start()
