#!/usr/bin/env python

from taskbase import TaskBase


class ToDepthTask(TaskBase):

    def __init__(self):
        super(ToDepthTask, self).__init__('to_depth')

    def run(self):
        result = self.move_to_point(self.state.pose.pose.position.x,
                                    self.state.pose.pose.position.y,
                                    -1.2)
        return self.FINISHED if result else self.CONTINUE
