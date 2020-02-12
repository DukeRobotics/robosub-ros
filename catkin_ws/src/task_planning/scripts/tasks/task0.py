#!/usr/bin/env python

from taskbase import TaskBase
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
import rospy

class Task0(TaskBase):

    def __init__(self):
        #rospy.Subscriber('/mavros/imu/data', Imu, self._print_imu)
        super(Task0, self).__init__('task0')

    def _print_imu(self, data):
        rpy = euler_from_quaternion([data.orientation.x, data.orientation.y,
                                     data.orientation.z, data.orientation.w])
        rospy.loginfo("Roll: " + str(rpy[0]))
        rospy.loginfo("Pitch: " + str(rpy[1]))
        rospy.loginfo("Yaw: " + str(rpy[2]))
        rospy.loginfo("")

    def run(self):
        rospy.loginfo(self.state.pose.pose)
        return self.CONTINUE
