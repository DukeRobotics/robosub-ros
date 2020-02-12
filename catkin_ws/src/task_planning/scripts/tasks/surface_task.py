#!/usr/bin/env python

from taskbase import TaskBase
import rospy


class SurfaceTask(TaskBase):

    def __init__(self):
        super(SurfaceTask, self).__init__('surface')

    def pre_run(self):
        self.global_target_pose = self.get_global_target_pose_from_task_start(0, 0, 5, 0, 0, 0)

    def run(self):
        result = self.move_to_point(self.global_target_pose)
        
        if rospy.Time.now() - self.time_start < rospy.Duration(8):
            return self.CONTINUE

        return self.FINISHED
