#!/usr/bin/env python

from taskbase import TaskBase
import rospy

class SpinTask(TaskBase):

    TURN_TIME = 3

    def __init__(self):
        super(SpinTask, self).__init__('spin')
        self.state_num = 0

    def pre_run(self):
        self.global_target_pose_0 = self.get_global_target_pose_from_task_start(0, 0, 0, 0, 0, 0)
        self.global_target_pose_1 = self.get_global_target_pose_from_task_start(0, 0, 0, 0, 0, 2.09)
        self.global_target_pose_2 = self.get_global_target_pose_from_task_start(0, 0, 0, 0, 0, 4.19)

    def run(self):

        result = None
        if self.state_num is 0:
            result = self.move_to_point(self.global_target_pose_1)
        elif self.state_num is 1:
            result = self.move_to_point(self.global_target_pose_2)
        elif self.state_num is 2:
            result = self.move_to_point(self.global_target_pose_0)
        elif self.state_num is 3:
            result = self.move_to_point(self.global_target_pose_1)
        elif self.state_num is 4:
            result = self.move_to_point(self.global_target_pose_2)
        elif self.state_num is 5:
            result = self.move_to_point(self.global_target_pose_0)

        # Update state
        if result or rospy.Time.now() - (self.time_start + rospy.Duration(self.turn_time_for_state() * self.state_num)) > rospy.Duration(self.turn_time_for_state()):
            if self.state_num is 5:
                return self.FINISHED
            self.state_num += 1
            return self.CONTINUE

        return self.CONTINUE

    def turn_time_for_state(self):
        if self.state_num != 5:
            return self.TURN_TIME
        else:
            return 15
