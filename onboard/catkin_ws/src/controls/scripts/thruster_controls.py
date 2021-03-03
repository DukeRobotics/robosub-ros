#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from custom_msgs.msg import ThrusterSpeeds
import numpy as np
from thruster_manager import ThrusterManager
from std_srvs.srv import SetBool
import controls_utils as utils
import resource_retriever as rr


class ThrusterController:
    ROBOT_PUB_TOPIC = '/offboard/thruster_speeds'

    enabled = False

    def __init__(self):
        rospy.init_node('thruster_controls')

        self.pub = rospy.Publisher(self.ROBOT_PUB_TOPIC, ThrusterSpeeds, queue_size=3)

        self.enable_service = rospy.Service('enable_controls', SetBool, self.handle_enable_controls)

        self.tm = ThrusterManager(rr.get_filename('package://controls/config/cthulhu.config', use_protocol=False))

        self.pid_outputs = np.zeros(6)
        self.pid_outputs_local = np.zeros(6)
        self.powers = np.zeros(6)
        self.t_allocs = np.zeros(8)

        for d in utils.get_axes():
            rospy.Subscriber(utils.get_controls_move_topic(d), Float64, self._on_pid_received, d)
            rospy.Subscriber(utils.get_power_topic(d), Float64, self._on_power_received, d)

    def handle_enable_controls(self, req):
        self.enabled = req.data
        return {'success': True, 'message': 'Successfully set enabled to ' + str(req.data)}

    def _on_pid_received(self, val, direction):
        self.pid_outputs[utils.get_axes().index(direction)] = val.data
        self.t_allocs = self.tm.calc_t_allocs(self.pid_outputs)

    def _on_power_received(self, val, direction):
        if val.data != 0:
            self.pid_outputs[utils.get_axes().index(direction)] = val.data
        self.t_allocs = self.tm.calc_t_allocs(self.pid_outputs)

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            if not self.enabled:
                # If not enabled, publish all 0s.
                i8_t_allocs = ThrusterSpeeds()
                i8_t_allocs.speeds = np.zeros(8)
                self.pub.publish(i8_t_allocs)

            if self.enabled:
                # Scale thruster alloc max to PID max
                t_alloc_max = float(np.max(np.absolute(self.t_allocs)))
                pid_max = float(np.max(np.absolute(self.pid_outputs)))

                if t_alloc_max != 0:
                    # Multiply each thruster allocation by scaling ratio
                    self.t_allocs *= pid_max / t_alloc_max
                # Clamp values of t_allocs to between -1 to 1
                self.t_allocs = np.clip(self.t_allocs, -1, 1)

                i8_t_allocs = ThrusterSpeeds()
                i8_t_allocs.speeds = (self.t_allocs * 60).astype(int)
                self.pub.publish(i8_t_allocs)

            rate.sleep()


def main():
    try:
        ThrusterController().run()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
