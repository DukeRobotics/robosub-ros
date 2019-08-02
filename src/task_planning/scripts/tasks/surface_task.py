#!/usr/bin/env python

from taskbase import TaskBase


class SurfaceTask(TaskBase):

    def __init__(self):
        super(SurfaceTask, self).__init__('surface')

    def run(self):
        result = self.move_to_point(self.state.pose.pose.position.x,
                                    self.state.pose.pose.position.y,
                                    0)
        return self.FINISHED if result else self.CONTINUE