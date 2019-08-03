#!/usr/bin/env python

from taskbase import TaskBase
import rospy

class SpinTask(TaskBase):

    TURN_TIME = 6

    def __init__(self):
        super(SpinTask, self).__init__('spin')
        self.state = 0

    def pre_run(self):
        self.global_target_pose_0 = self.get_global_target_pose_from_task_start(0, 0, 0, 0, 0, 0)
        self.global_target_pose_1 = self.get_global_target_pose_from_task_start(0, 0, 0, 0, 0, 2.09)
        self.global_target_pose_2 = self.get_global_target_pose_from_task_start(0, 0, 0, 0, 0, 4.19)

    def run(self):

        if self.state is 0:
            result = self.move_to_point(self.global_target_pose_1)
        elif self.state is 1:
            result = self.move_to_point(self.global_target_pose_2)
        elif self.state is 2:
            result = self.move_to_point(self.global_target_pose_0)
        elif self.state is 3:
            result = self.move_to_point(self.global_target_pose_1)
        elif self.state is 4:
            result = self.move_to_point(self.global_target_pose_2)
        elif self.state is 5:
            result = self.move_to_point(self.global_target_pose_0)

        # Update state
        if result or rospy.Time.now() - (self.time_start + rospy.Duration(self.TURN_TIME * self.state)) > self.TURN_TIME:
            if self.state is 5:
                return self.FINISHED
            self.state += 1
            return self.CONTINUE

        return self.CONTINUE
