#!/usr/bin/env python

import rospy

from competition_task import CompetitionTask


class TaskRunner:
    RATE = 30  # Hz

    def __init__(self):
        rospy.init_node("task_planning")
        self.competition_task = CompetitionTask()

    def start(self):
        rate = rospy.Rate(self.RATE)

        while not self.competition_task.finished:
            self.competition_task.run()
            rate.sleep()


TaskRunner().start()
