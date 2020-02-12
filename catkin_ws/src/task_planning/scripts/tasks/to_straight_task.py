#!/usr/bin/env python

from taskbase import TaskBase
import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Quaternion
import tf2_geometry_msgs
from tf.transformations import quaternion_from_euler, euler_from_quaternion

class ToStraightTask(TaskBase):

    DISTANCE = 1

    def __init__(self):
        super(ToStraightTask, self).__init__('to_straight')

    def pre_run(self):
        local_target_pose = PoseStamped()
        #local_target_pose.pose.position.x = self.DISTANCE
        #local_target_pose.pose.position.y = self.DISTANCE
        #local_target_pose.pose.position.z = -self.DISTANCE
        local_target_pose.header = Header(stamp=rospy.Time.now(), frame_id='base_link')
        local_target_pose.pose.orientation = Quaternion(*quaternion_from_euler(0,0,45))
        self.global_target_pose = tf2_geometry_msgs.do_transform_pose(local_target_pose,
                                               self.task_start_transform_to_global)
        rospy.loginfo(self.task_start_transform_to_global)
        rospy.loginfo(local_target_pose)
        rospy.loginfo(self.global_target_pose)

    def run(self):
        rospy.loginfo("here")
        #if rospy.Time.now() - self.time_start < rospy.Duration(10):
        #    return self.CONTINUE

        result = self.move_to_point(self.global_target_pose)
        return self.FINISHED if result else self.CONTINUE
