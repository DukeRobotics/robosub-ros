#!/usr/bin/env python

import rospy

from competition_task import CompetitionTask


class TaskRunner:

    def __init__(self):
        rospy.init_node("task_planning")
        self.competition_task = CompetitionTask()

    def start(self):
        rate = 15 #Hz
        self.rate = rospy.Rate(rate)

        while not self.competition_task.finished:
            self.competition_task.run()
            self.rate.sleep()


TaskRunner().start()
