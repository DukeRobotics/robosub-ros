#!/usr/bin/env python
import sys
import os

import rospy
from std_msgs.msg import Float64, Float32MultiArray, Int8MultiArray
from controls.msg import ThrusterSpeeds
import numpy as np
from thruster_manager import ThrusterManager
from std_srvs.srv import SetBool

class ThrusterController():

    CONTROLS_MOVE_TOPIC = '/control_effort'
    CONTROLS_MOVE_X_TOPIC = CONTROLS_MOVE_TOPIC + '/x'
    CONTROLS_MOVE_Y_TOPIC = CONTROLS_MOVE_TOPIC + '/y'
    CONTROLS_MOVE_Z_TOPIC = CONTROLS_MOVE_TOPIC + '/z'
    CONTROLS_MOVE_ROLL_TOPIC = CONTROLS_MOVE_TOPIC + '/roll'
    CONTROLS_MOVE_PITCH_TOPIC = CONTROLS_MOVE_TOPIC + '/pitch'
    CONTROLS_MOVE_YAW_TOPIC = CONTROLS_MOVE_TOPIC + '/yaw'

    SIM_PUB_TOPIC = '/sim/move'
    ROBOT_PUB_TOPIC = '/offboard/thruster_speeds'

    enabled = False

    def __init__(self):
        self.sim = rospy.get_param('~/thruster_controls/sim')
        if self.sim == False:
            self.pub = rospy.Publisher(self.ROBOT_PUB_TOPIC, ThrusterSpeeds, queue_size=3)
        elif self.sim == True:
            self.pub = rospy.Publisher(self.SIM_PUB_TOPIC, Float32MultiArray, queue_size=3)
        else:
            # TODO: alert that unrecognized mode destination has been set
            pass

        self.enable_service = rospy.Service('enable_controls', SetBool, self.handle_enable_controls)

        self.tm = ThrusterManager(os.path.join(sys.path[0], '../config/cthulhu.config'))

        rospy.Subscriber(self.CONTROLS_MOVE_X_TOPIC, Float64, self._on_x)
        rospy.Subscriber(self.CONTROLS_MOVE_Y_TOPIC, Float64, self._on_y)
        rospy.Subscriber(self.CONTROLS_MOVE_Z_TOPIC, Float64, self._on_z)
        rospy.Subscriber(self.CONTROLS_MOVE_ROLL_TOPIC, Float64, self._on_roll)
        rospy.Subscriber(self.CONTROLS_MOVE_PITCH_TOPIC, Float64, self._on_pitch)
        rospy.Subscriber(self.CONTROLS_MOVE_YAW_TOPIC, Float64, self._on_yaw)

        self.pid_outputs = np.zeros(6)
        self.t_allocs = np.zeros(8)

    def handle_enable_controls(self, req):
        self.enabled = req.data
        return {'success': True, 'message': 'Successfully set enabled to ' + str(req.data)}

    def update_thruster_allocs(self):
        # Calculate thruster allocations
        self.t_allocs = self.tm.calc_t_allocs(self.pid_outputs)

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

    def run(self):
        rospy.init_node('thruster_controls')
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
                pid_max = float(np.max(np.absolute(self.pid_outputs)))

                if(t_alloc_max != 0):
                    # Multiply each thruster allocation by scaling ratio
                    self.t_allocs *= pid_max / t_alloc_max

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
