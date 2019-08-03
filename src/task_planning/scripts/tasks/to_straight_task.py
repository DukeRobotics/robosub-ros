#!/usr/bin/env python

from taskbase import TaskBase
import rospy
from geometry_msgs.msg import PoseStamped
import tf2_geometry_msgs

class ToStraightTask(TaskBase):

    DISTANCE = 2

    def __init__(self):
        super(ToStraightTask, self).__init__('to_straight')

    def pre_run(self):
        local_target_pose = PoseStamped()
        local_target_pose.pose.position.x = self.DISTANCE
        self.global_target_pose = tf2_geometry_msgs.do_transform_pose(local_target_pose,
                                                self.task_start_transform_to_global)
        rospy.loginfo(local_target_pose)
        rospy.loginfo(self.global_target_pose)

    def run(self):
        rospy.loginfo("here")
        #if rospy.Time.now() - self.time_start < rospy.Duration(10):
        #    return self.CONTINUE

        result = self.move_to_point(self.global_target_pose)
        return self.FINISHED if result else self.CONTINUE
