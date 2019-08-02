#!/usr/bin/env python

from taskbase import TaskBase


class ToGateTask(TaskBase):

    GATE_X = 5
    GATE_Y = 5
    GATE_Z = -1.2

    def __init__(self):
        super(ToGateTask, self).__init__('to_gate')

    def run(self):
        result = self.move_to_point(self.GATE_X,
                                    self.GATE_Y,
                                    self.GATE_Z)
        return self.FINISHED if result else self.CONTINUE
