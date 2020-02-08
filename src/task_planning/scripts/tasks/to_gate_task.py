#!/usr/bin/env python

from taskbase import TaskBase
from geometry_msgs.msg import PoseStamped
import tf2_geometry_msgs
import rospy


class ToGateTask(TaskBase):

    GATE_DIST = 13.5

    def __init__(self):
        super(ToGateTask, self).__init__('to_gate')

    def pre_run(self):
        self.global_target_pose = self.get_global_target_pose_from_task_start(self.GATE_DIST, 0, 0, 0, 0, 0)

    def run(self):
        if rospy.Time.now() - self.time_start > rospy.Duration(40):
            return self.FINISHED

        result = self.move_to_point(self.global_target_pose)
        return self.FINISHED if result else self.CONTINUE
