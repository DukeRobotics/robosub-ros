#!/usr/bin/env python

import rospy
import numpy as np
import scipy.linalg as linalg
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64, Float32MultiArray
from tf.transformations import euler_from_quaternion
from symbolic_state import SymbolicState
import control
import sys
import os


class lqrController:

    SIM_PUB_TOPIC = '/sim/move'

    def __init__(self):
        rospy.init_node("lqr")
        self.state_topic = "/state"
        self.desired_pose_topic = "controls/desired_pose"
        self.enable_topic = "/lqr/enable"
        rospy.Subscriber(self.enable_topic, Bool, self.receive_enable)
        rospy.Subscriber(self.state_topic, Odometry, self.receive_curr)
        rospy.Subscriber(self.desired_pose_topic, Pose, self.receive_desired_pose)
        self.pub = rospy.Publisher(self.SIM_PUB_TOPIC, Float32MultiArray, queue_size=3)
        self.enabled = False
        # state is array of [x, y, z, roll, pitch, yaw, u, v, w, p, q, r] (axes and their first derivative)
        self.curr_state = np.zeros(12)
        self.desired_state = np.zeros(12)
        f1 = os.path.join(sys.path[0], '../config/config.yaml')
        f2 = os.path.join(sys.path[0], '../config/cthulhu.config')
        self.symbolic_state = SymbolicState(f1, f2)

    def receive_enable(self, val):
        self.enabled = val.data

    def receive_curr(self, odom):
        self.curr_state = np.array(self.parse_pose(odom.pose.pose) + self.parse_twist(odom.twist.twist))

    def receive_desired_pose(self, desired):
        self.desired_state[:6] = self.parse_pose(desired)
        self.desired_state[6:] = np.zeros(6)

    def parse_pose(self, pose):
        pose_list = [pose.position.x, pose.position.y, pose.position.z]
        pose_list.extend(euler_from_quaternion([pose.orientation.x,
                                                pose.orientation.y,
                                                pose.orientation.z,
                                                pose.orientation.w]))
        return pose_list

    def parse_twist(self, twist):
        return [twist.linear.x, twist.linear.y, twist.linear.z,
                twist.angular.x, twist.angular.y, twist.angular.z]

    def run(self):

        while not rospy.is_shutdown():
            if True:  # self.enabled
                error = self.curr_state - self.desired_state
                error[3:6] = np.arctan2(np.sin(self.curr_state[3:6]-self.desired_state[3:6]),
                                        np.cos(self.curr_state[3:6]-self.desired_state[3:6]))

                A, B, Q, R = self.symbolic_state.get_state_space(self.curr_state)
                print(A)
                print(B)
                #X = linalg.solve_continuous_are(A, B, Q, R)
                #K = np.matmul(linalg.inv(R), np.matmul(B.T, X))
                try:
                    K, S, E = control.lqr(A, B, Q, R)
                    du = -np.matmul(K, error)
                except:
                    print("fail")
                    du = np.zeros(8)
                print(du)
                f32_t_allocs = Float32MultiArray()
                f32_t_allocs.data = list(du)
                self.pub.publish(f32_t_allocs)

if __name__ == '__main__':
    lqrController().run()