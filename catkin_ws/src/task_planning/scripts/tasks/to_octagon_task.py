#!/usr/bin/env python

from taskbase import TaskBase


class ToOctagonTask(TaskBase):

    OCT_X = 15
    OCT_Y = 15
    OCT_Z = -1.2

    def __init__(self):
        super(ToOctagonTask, self).__init__('to_octagon')

    def run(self):
        result = self.move_to_point(self.OCT_X,
                                    self.OCT_Y,
                                    self.OCT_Z)
        return self.FINISHED if result else self.CONTINUE