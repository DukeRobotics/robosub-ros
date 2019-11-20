#!/usr/bin/env python

import json
import numpy as np
from thruster import Thruster

class ThrusterManager():

    def __init__(self, config_filename):
        with open(config_filename) as f:
            self.vehicle = json.load(f)

        self.thrusters = []
        for t_dict in self.vehicle['thrusters']:
            t = Thruster(t_dict['pos'], t_dict['rpy'])
            self.thrusters.append(t)
        
        self.wrenchmat = np.empty((6, len(self.thrusters)))
        for i, t in enumerate(self.thrusters):
            self.wrenchmat[0][i] = t.force_hat[0]
            self.wrenchmat[1][i] = t.force_hat[1]
            self.wrenchmat[2][i] = t.force_hat[2]
            self.wrenchmat[3][i] = t.torque[0]
            self.wrenchmat[4][i] = t.torque[1]
            self.wrenchmat[5][i] = t.torque[2]

        self.wrenchmat_pinv = np.linalg.pinv(self.wrenchmat)


    def subscribe_to_pid(self):
        # TODO: actually subscribe to PID messages from ROS
        # pid = np.random.rand(6)
        pid = [1, 0, 0, 0, 0, 1]

        thruster_allocations = self.calc_thruster_allocs(pid)
        print(np.linalg.norm(thruster_allocations))
        return thruster_allocations
        # TODO: publish these bad boys

    def calc_thruster_allocs(self, pid_wrench):
        thruster_allocations = np.matmul(self.wrenchmat_pinv, pid_wrench)

        max_pow = np.max(np.abs(thruster_allocations))
        if max_pow > 1:
            thruster_allocations /= max_pow

        return thruster_allocations


if __name__ == '__main__':
    tm = ThrusterManager('cthulhu.config')
    print(tm.wrenchmat)
    print(tm.wrenchmat_pinv)
    print(tm.subscribe_to_pid())
