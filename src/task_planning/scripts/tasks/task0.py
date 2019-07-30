#!/usr/bin/env python

from taskbase import TaskBase

class Task0(TaskBase):

    def __init__(self):
        super(Task0, self).__init__('task0')

    def run(self):
        print('yo yo')
        return self.FINISHED
