#!/usr/bin/env python

from taskbase import TaskBase
import rospy


class ToDepthTask(TaskBase):

    def __init__(self):
        super(ToDepthTask, self).__init__('to_depth')

    def run(self):
        rospy.loginfo("here")
        if rospy.Time.now() - self.time_start < rospy.Duration(10):
            return self.CONTINUE

        result = self.move_to_point(self.state.pose.pose.position.x,
                                    self.state.pose.pose.position.y,
                                    -0.1)
        return self.FINISHED if result else self.CONTINUE
