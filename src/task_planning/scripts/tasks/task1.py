#!/usr/bin/env python

from taskbase import TaskBase

class Task1(TaskBase):
    
    def __init__(self):
        super(Task1, self).__init__('task1')
    
    def run(self):
        print('Wazzup')
