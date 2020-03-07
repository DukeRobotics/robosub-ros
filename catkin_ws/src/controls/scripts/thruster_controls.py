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

class ThrusterController():

    CONTROLS_MOVE_TOPIC = '/control_effort'
    CONTROLS_MOVE_X_TOPIC = CONTROLS_MOVE_TOPIC + '/x'
    CONTROLS_MOVE_Y_TOPIC = CONTROLS_MOVE_TOPIC + '/y'
    CONTROLS_MOVE_Z_TOPIC = CONTROLS_MOVE_TOPIC + '/z'
    CONTROLS_MOVE_ROLL_TOPIC = CONTROLS_MOVE_TOPIC + '/roll'
    CONTROLS_MOVE_PITCH_TOPIC = CONTROLS_MOVE_TOPIC + '/pitch'
    CONTROLS_MOVE_YAW_TOPIC = CONTROLS_MOVE_TOPIC + '/yaw'

    POWER_TOPIC_X = '/controls/power/x'
    POWER_TOPIC_Y = '/controls/power/y'
    POWER_TOPIC_Z = '/controls/power/z'
    POWER_TOPIC_ROLL = '/controls/power/roll'
    POWER_TOPIC_PITCH = '/controls/power/pitch'
    POWER_TOPIC_YAW = '/controls/power/yaw'

    SIM_PUB_TOPIC = '/sim/move'
    ROBOT_PUB_TOPIC = '/offboard/thruster_speeds'

    enabled = False

    def __init__(self):
        rospy.init_node('thruster_controls')

        self.sim = rospy.get_param('~/thruster_controls/sim')
        if self.sim == False:
            self.pub = rospy.Publisher(self.ROBOT_PUB_TOPIC, ThrusterSpeeds, queue_size=3)
        elif self.sim == True:
            self.pub = rospy.Publisher(self.SIM_PUB_TOPIC, Float32MultiArray, queue_size=3)

        self.enable_service = rospy.Service('enable_controls', SetBool, self.handle_enable_controls)

        self.tm = ThrusterManager(os.path.join(sys.path[0], '../config/cthulhu.config'))

        self.listener = TransformListener()

        rospy.Subscriber(self.CONTROLS_MOVE_X_TOPIC, Float64, self._on_x)
        rospy.Subscriber(self.CONTROLS_MOVE_Y_TOPIC, Float64, self._on_y)
        rospy.Subscriber(self.CONTROLS_MOVE_Z_TOPIC, Float64, self._on_z)
        rospy.Subscriber(self.CONTROLS_MOVE_ROLL_TOPIC, Float64, self._on_roll)
        rospy.Subscriber(self.CONTROLS_MOVE_PITCH_TOPIC, Float64, self._on_pitch)
        rospy.Subscriber(self.CONTROLS_MOVE_YAW_TOPIC, Float64, self._on_yaw)

        rospy.Subscriber(self.POWER_TOPIC_X, Float64, self._on_x_power)
        rospy.Subscriber(self.POWER_TOPIC_Y, Float64, self._on_y_power)
        rospy.Subscriber(self.POWER_TOPIC_Z, Float64, self._on_z_power)
        rospy.Subscriber(self.POWER_TOPIC_ROLL, Float64, self._on_roll_power)
        rospy.Subscriber(self.POWER_TOPIC_PITCH, Float64, self._on_pitch_power)
        rospy.Subscriber(self.POWER_TOPIC_YAW, Float64, self._on_yaw_power)

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
            if(self.powers[i]!=0):
                self.pid_outputs_local[i]=self.powers[i]

        self.t_allocs = self.tm.calc_t_allocs(self.pid_outputs_local)


    def _on_x(self, x):
        self.pid_outputs[0] = x.data
        self.update_thruster_allocs()

    def _on_y(self, y):
        self.pid_outputs[1] = y.data
        self.update_thruster_allocs()

    def _on_z(self, z):
        self.pid_outputs[2] = z.data
        self.update_thruster_allocs()

    def _on_roll(self, roll):
        self.pid_outputs[3] = roll.data
        self.update_thruster_allocs()

    def _on_pitch(self, pitch):
        self.pid_outputs[4] = pitch.data
        self.update_thruster_allocs()

    def _on_yaw(self, yaw):
        self.pid_outputs[5] = yaw.data
        self.update_thruster_allocs()

    def _on_x_power(self, x):
        self.powers[0] = x.data
        self.update_thruster_allocs()

    def _on_y_power(self, y):
        self.powers[1] = y.data
        self.update_thruster_allocs()

    def _on_z_power(self, z):
        self.powers[2] = z.data
        self.update_thruster_allocs()

    def _on_roll_power(self, roll):
        self.powers[3] = roll.data
        self.update_thruster_allocs()

    def _on_pitch_power(self, pitch):
        self.powers[4] = pitch.data
        self.update_thruster_allocs()

    def _on_yaw_power(self, yaw):
        self.powers[5] = yaw.data
        self.update_thruster_allocs()

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            if not self.enabled:
                # If not enabled, publish all 0s.
                if self.sim == False:
                    i8_t_allocs = ThrusterSpeeds()
                    i8_t_allocs.speeds = np.zeros(8)
                    self.pub.publish(i8_t_allocs)
                elif self.sim == True:
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

                if self.sim == False:
                    i8_t_allocs = ThrusterSpeeds()
                    i8_t_allocs.speeds = (self.t_allocs * 127).astype(int)
                    self.pub.publish(i8_t_allocs)
                elif self.sim == True:
                    f32_t_allocs = Float32MultiArray()
                    f32_t_allocs.data = self.t_allocs
                    self.pub.publish(f32_t_allocs)

            rate.sleep()


def main():
    try:
        ThrusterController().run()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
