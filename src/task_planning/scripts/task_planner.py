#!/usr/bin/env python
import json
from pprint import pprint
import sys
import rospy
import importlib

class TaskPlanner:
    
    PLANS_FILE = '../plan/plans.json'
    NODE_NAME = 'task_planner'
    
    # REFACTOR THIS
    CONTINUE = 1
    FINISHED = 2
    
    def __init__(self):
        with open(self.PLANS_FILE) as plans_file:
            self.masterplan= json.load(plans_file)
        
        self.plan_name = sys.argv[1]
        
        self.init_tasks(self.masterplan)
        self.plan = self.init_plan(self.masterplan, self.plan_name)
        
        rospy.init_node(self.NODE_NAME)
        
    def init_tasks(self, masterplan):
        self.tasks = []
        sys.path.append('./tasks')
        for task_info in masterplan['tasks']:
            task = getattr(importlib.import_module(task_info['modulename']), task_info['classname'])()
            self.tasks.append(task)
    
    def init_plan(self, masterplan, plan_name):
        target_plan = None
        for plan in masterplan['plans']:
            if plan['name'] == plan_name:
                target_plan = plan
                break
        
        if target_plan == None:
            raise Exception('Plan ' + plan_name + ' not found')
        
        task_names = target_plan['tasks']
        self.tasks_plan = map(self._get_task_from_name, task_names)
    
    def _get_task_from_name(self, name):
        return list(filter(lambda task: task.name == name, self.tasks))[0]
    
    def run(self):
        rate = rospy.Rate(10)
        for task in self.tasks_plan:
            while not rospy.is_shutdown():
                result = task.run()
                if result == self.CONTINUE:
                    continue
                elif result == self.FINISHED:
                    break

if __name__ == '__main__':
    TaskPlanner().run()
