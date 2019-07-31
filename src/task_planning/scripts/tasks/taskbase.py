#!/usr/bin/env python

from enum import Enum

class TaskBase(object):

    CONTINUE = 1
    FINISHED = 2
    
    def __init__(self, name):
        self.name = name
    
    def run(self):
        raise NotImplementedError('Run method of task not implemented, or running base class')

class TaskResult(Enum):
    CONTINUE = 1
    FINISHED = 2
