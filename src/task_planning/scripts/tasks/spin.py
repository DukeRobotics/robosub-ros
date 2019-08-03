#!/usr/bin/env python

from taskbase import TaskBase
import rospy

class SpinTask(TaskBase):

    def __init__(self):
        super(SpinTask, self).__init__('spin')

    def pre_run(self):
        self.global_target_pose = self.get_global_target_pose_from_task_start(0, 0, 0, 0, 0, 2.09)

    def run(self):
        result = self.move_to_point(self.global_target_pose)

        if result:
            return self.FINISHED

        if rospy.Time.now() - self.time_start > rospy.Duration(6):
            return self.FINISHED

        return self.CONTINUE
