#!/usr/bin/env python

from taskbase import TaskBase


class ToBuoyTask(TaskBase):

    BUOY_X = 10
    BUOY_Y = 10
    BUOY_Z = -1.2

    def __init__(self):
        super(ToBuoyTask, self).__init__('to_buoy')

    def run(self):
        result = self.move_to_point(self.BUOY_X,
                                    self.BUOY_Y,
                                    self.BUOY_Z)
        return self.FINISHED if result else self.CONTINUE
