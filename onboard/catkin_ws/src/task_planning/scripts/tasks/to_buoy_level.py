#!/usr/bin/env python

from taskbase import TaskBase
import rospy


class ToBuoyLevelTask(TaskBase):

    DELTA_Z = -0.7

    def __init__(self):
        super(ToBuoyLevelTask, self).__init__('buoylevel')

    def pre_run(self):
        self.global_target_pose = self.get_global_target_pose_from_task_start(0, 0, self.DELTA_Z, 0, 0, 0)

    def run(self):
        if rospy.Time.now() - self.time_start > rospy.Duration(6):
            return self.FINISHED

        result = self.move_to_point(self.global_target_pose)
        return self.FINISHED if result else self.CONTINUE
