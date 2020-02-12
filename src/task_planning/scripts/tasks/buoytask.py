#!/usr/bin/env python

from taskbase import TaskBase
import rospy


class BuoyTask(TaskBase):

    CYCLE_TIME = 8

    def __init__(self):
        super(BuoyTask, self).__init__('buoytask')
        self.hit_points = [[0.77, 1.85], [1.41, 1.41], [1.77, 0.92], [2, 0], [1.77, -0.92], [1.41, -1.41], [0.77, -1.85]]  # (x, y) where x is forwards
        self.global_start_pose = None
        self.global_point_poses = []
        self.curr_point = 0
        self.charging = True

    def pre_run(self):
        self.global_start_pose = self.get_global_target_pose_from_task_start(0, 0, 0, 0, 0, 0)
        for point in self.hit_points:
            self.global_point_poses.append(self.get_global_target_pose_from_task_start(point[0], point[1], 0, 0, 0, 0))

    def run(self):
        result = None

        # Move
        if self.charging:
            result = self.move_to_point(self.global_point_poses[self.curr_point])
        else:
            result = self.move_to_point(self.global_start_pose)

        # Update state
        if result or rospy.Time.now() - (self.time_start + rospy.Duration(self.CYCLE_TIME * self.time_for_state())) > rospy.Duration(self.CYCLE_TIME):
            if self.charging:
                self.charging = False
                self.curr_point += 1
                return self.CONTINUE
            else:
                self.charging = True
                if self.curr_point == len(self.global_point_poses):
                    return self.FINISHED
                else:
                    return self.CONTINUE

        return self.CONTINUE

    def time_for_state(self):
        return self.curr_point * 2 if self.charging else (self.curr_point * 2) - 1
