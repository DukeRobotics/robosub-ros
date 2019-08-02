#!/usr/bin/env python

from taskbase import TaskBase
import rospy


class ToDepthTask(TaskBase):

    DEPTH = -1

    def __init__(self):
        super(ToDepthTask, self).__init__('to_depth')

    def pre_run(self):
        local_target_pose = PoseStamped()
        local_target_pose.position.z = self.DEPTH
        self.global_target_pose = tf2_geometry_msgs.do_transform_pose(local_target_pose,
                                                self.task_start_transform_to_global)

    def run(self):
        rospy.loginfo("here")
        #if rospy.Time.now() - self.time_start < rospy.Duration(10):
        #    return self.CONTINUE

        result = self.move_to_point(self.global_target_pose)
        return self.FINISHED if result else self.CONTINUE
