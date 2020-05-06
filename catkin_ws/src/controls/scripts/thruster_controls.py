#!/usr/bin/env python
import sys
import os

import rospy
from std_msgs.msg import Float64, Float32MultiArray, Int8MultiArray
from geometry_msgs.msg import Vector3Stamped
from controls.msg import ThrusterSpeeds
import numpy as np
from thruster_manager import ThrusterManager
from std_srvs.srv import SetBool
from tf import TransformListener
import drc_utils as utils

class ThrusterController():
    DIRECTIONS = ['x', 'y', 'z', 'roll', 'pitch', 'yaw']

    SIM_PUB_TOPIC = '/sim/move'
    ROBOT_PUB_TOPIC = '/offboard/thruster_speeds'

    enabled = False

    def __init__(self):
        rospy.init_node('thruster_controls')

        self.sim = rospy.get_param('~/thruster_controls/sim')
        if not self.sim :
            self.pub = rospy.Publisher(self.ROBOT_PUB_TOPIC, ThrusterSpeeds, queue_size=3)
        else:
            self.pub = rospy.Publisher(self.SIM_PUB_TOPIC, Float32MultiArray, queue_size=3)

        self.enable_service = rospy.Service('enable_controls', SetBool, self.handle_enable_controls)

        self.tm = ThrusterManager(os.path.join(sys.path[0], '../config/cthulhu.config'))

        self.listener = TransformListener()

        for d in self.DIRECTIONS:
            rospy.Subscriber(utils.get_controls_move_topic(d), Float64, self._on, d)
            rospy.Subscriber(utils.get_power_topic(d), Float64, self._on_power, d)

        self.pid_outputs = np.zeros(6)
        self.pid_outputs_local = np.zeros(6)
        self.powers = np.zeros(6)
        self.t_allocs = np.zeros(8)

    def handle_enable_controls(self, req):
        self.enabled = req.data
        return {'success': True, 'message': 'Successfully set enabled to ' + str(req.data)}

    def transform_twist(self, base_frame, target_frame, twist):
        lin = Vector3Stamped()
        ang = Vector3Stamped()

        lin.header.frame_id = base_frame
        ang.header.frame_id = base_frame

        lin.vector.x = twist[0]
        lin.vector.y = twist[1]
        lin.vector.z = twist[2]
        ang.vector.x = twist[3]
        ang.vector.y = twist[4]
        ang.vector.z = twist[5]

        lin_local = self.listener.transformVector3(target_frame, lin)
        ang_local = self.listener.transformVector3(target_frame, ang)

        return np.array([lin_local.vector.x, 
                         lin_local.vector.y, 
                         lin_local.vector.z,
                         ang_local.vector.x,
                         ang_local.vector.y,
                         ang_local.vector.z])

    def update_thruster_allocs(self):
        if self.enabled:
            self.pid_outputs_local = self.transform_twist('odom', 'base_link', self.pid_outputs)

        for i in range(len(self.powers)):
            if self.powers[i] != 0:
                self.pid_outputs_local[i] = self.powers[i]

        self.t_allocs = self.tm.calc_t_allocs(self.pid_outputs_local)

    def _on(self, val, direction):
        self.pid_outputs[self.DIRECTIONS.index(direction)] = val.data
        self.update_thruster_allocs()

    def _on_power(self, val, direction):
        self.powers[self.DIRECTIONS.index(direction)] = val.data
        self.update_thruster_allocs()

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            if not self.enabled:
                # If not enabled, publish all 0s.
                if not self.sim:
                    i8_t_allocs = ThrusterSpeeds()
                    i8_t_allocs.speeds = np.zeros(8)
                    self.pub.publish(i8_t_allocs)
                else:
                    f32_t_allocs = Float32MultiArray()
                    f32_t_allocs.data = np.zeros(8)
                    self.pub.publish(f32_t_allocs)

            if self.enabled:
                # Scale thruster alloc max to PID max
                t_alloc_max = float(np.max(np.absolute(self.t_allocs)))
                pid_max = float(np.max(np.absolute(self.pid_outputs_local)))

                if(t_alloc_max != 0):
                    # Multiply each thruster allocation by scaling ratio
                    self.t_allocs *= pid_max / t_alloc_max
                #Clamp values of t_allocs to between -1 to 1
                self.t_allocs = np.clip(self.t_allocs, -1 , 1)

                if not self.sim:
                    i8_t_allocs = ThrusterSpeeds()
                    i8_t_allocs.speeds = (self.t_allocs * 127).astype(int)
                    self.pub.publish(i8_t_allocs)
                else:
                    f32_t_allocs = Float32MultiArray()
                    f32_t_allocs.data = self.t_allocs
                    self.pub.publish(f32_t_allocs)

            rate.sleep()

if __name__ == '__main__':
    try:
        ThrusterController().run()
    except rospy.ROSInterruptException:
        pass
