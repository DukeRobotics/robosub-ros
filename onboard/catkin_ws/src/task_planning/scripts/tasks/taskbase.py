#!/usr/bin/env python
from enum import Enum
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import Header
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf2_geometry_msgs
import rospy
import numpy as np
import tf2_ros

class TaskBase(object):

    CONTINUE = 1
    FINISHED = 2

    AT_POINT_MARGIN = 0.05
    AT_ANGLE_MARGIN = 0.2
    
    def __init__(self, name):
        self.name = name
        self.state = Odometry()
        rospy.Subscriber('/state', Odometry, self._receive_state)
        self._desired_state_pub = rospy.Publisher('/motion_planning/desired_state_global', PoseStamped, queue_size=10)
        self._tfBuffer = tf2_ros.Buffer()
        self._tfListener = tf2_ros.TransformListener(self._tfBuffer)
        #self.mission_start_transform_to_global = self._tfBuffer.lookup_transform('odom', 'base_link', rospy.Time(0), rospy.Duration(0.5))

    def _receive_state(self, msg):
        self.state = msg
    
    def run(self):
        raise NotImplementedError('run method of task not implemented, or running base class')

    def pre_run_base(self):
        self.time_start = rospy.Time.now()
        self.task_start_transform_to_global = self._tfBuffer.lookup_transform('odom', 'base_link', rospy.Time(0), rospy.Duration(0.5))

    def pre_run(self):
        pass

    def move_to_point(self, location):
        if self.dist_from_self(location.pose.position.x,
                               location.pose.position.y,
                               location.pose.position.z) < self.AT_POINT_MARGIN and self.angle_error_ok(location.pose.orientation):
            return True

        self._desired_state_pub.publish(location)
        return False

    def dist_from_self(self, x, y, z):
        self_point = np.array([self.state.pose.pose.position.x, self.state.pose.pose.position.y, self.state.pose.pose.position.z])
        other_point = np.array([x, y, z])
        return np.linalg.norm(other_point - self_point)
    
    def angle_error_ok(self, desired_orientation):
        goal_rpy = euler_from_quaternion([desired_orientation.x, desired_orientation.y, desired_orientation.z, desired_orientation.w])
        self_rpy = euler_from_quaternion([self.state.pose.pose.orientation.x, self.state.pose.pose.orientation.y, self.state.pose.pose.orientation.z, self.state.pose.pose.orientation.w])
        errors = np.array(goal_rpy) - np.array(self_rpy)
        for err in errors:
            if abs(err) > self.AT_ANGLE_MARGIN:
                return False
        return True

    def get_global_target_pose_from_task_start(self, x, y, z, roll, pitch, yaw):
        local_target_pose = PoseStamped()
        local_target_pose.header = Header(stamp=rospy.Time.now(), frame_id='base_link')
        local_target_pose.pose.orientation = Quaternion(*quaternion_from_euler(roll, pitch, yaw))
        local_target_pose.pose.position.x = x
        local_target_pose.pose.position.y = y
        local_target_pose.pose.position.z = z
        return tf2_geometry_msgs.do_transform_pose(local_target_pose, self.task_start_transform_to_global)


     #def get_global_target_pose_from_mission_start(self, x, y, z, roll, pitch, yaw):
     #   local_target_pose = PoseStamped()
     #   local_target_pose.header = Header(stamp=rospy.Time.now(), frame_id='base_link')
     #   local_target_pose.pose.orientation = Quaternion(*quaternion_from_euler(roll, pitch, yaw))
     #   local_target_pose.pose.position.x = x
     #   local_target_pose.pose.position.y = y
     #   local_target_pose.pose.position.z = z
     #   return tf2_geometry_msgs.do_transform_pose(local_target_pose,
     #                                              self.mission_start_transform_to_global)
#
#                                              self.task_start_transform_to_global)
#
#
#class TaskResult(Enum):
#    CONTINUE = 1
#    FINISHED = 2
