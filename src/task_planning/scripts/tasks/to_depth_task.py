#!/usr/bin/env python

from taskbase import TaskBase
import rospy


class ToDepthTask(TaskBase):

    DEPTH = -0.4

    def __init__(self):
        super(ToDepthTask, self).__init__('to_depth')

    def pre_run(self):
        self.global_target_pose = self.get_global_target_pose_from_task_start(0, 0, self.DEPTH, 0, 0, 0)

    def run(self):
        if rospy.Time.now() - self.time_start > rospy.Duration(10):
            return self.FINISHED

        result = self.move_to_point(self.global_target_pose)
        return self.FINISHED if result else self.CONTINUE
