#!/usr/bin/env python

from taskbase import TaskBase
from geometry_msgs.msg import PoseStamped

class ToGateTask(TaskBase):

    GATE_X = 5
    GATE_Y = 5
    GATE_Z = -1.2
    GATE_DIST = 3 #15

    def __init__(self):
        super(ToGateTask, self).__init__('to_gate')

    def pre_run(self):
        transform = self._tfBuffer.lookup_transform('odom', 'base_link', rospy.Time(0), rospy.Duration(0.5))
        self.want_pose = PoseStamped()
        self.want_pose.pose.position.x = self.GATE_DIST
        self.global_want_pose = tf2_geometry_msgs.do_transform_pose(self.want_pose,
                                     self._to_robot_transform)
        super(ToGateTask, self).pre_run()

    def run(self):
        result = self.move_to_point(self.global_want_pose.pose.position.x,
                                    self.global_want_pose.pose.position.y,
                                    self.global_want_pose.pose.position.z)
        return self.FINISHED if result else self.CONTINUE
