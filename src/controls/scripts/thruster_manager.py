#!/usr/bin/env python

import yaml
import numpy as np
from thruster import Thruster

class ThrusterManager():

    def __init__(self, config_filename):
        # Load vehicle configuration file
        with open(config_filename) as f:
            self.vehicle = yaml.load(f)

        # Initialize thrusters based off of configuration file
        self.thrusters = []
        for t_dict in self.vehicle['thrusters']:
            t = Thruster(t_dict['pos'], t_dict['rpy'])
            self.thrusters.append(t)

        # Creating wrench matrix with column vectors equal to the force and torque of each thruster
        self.wrenchmat = np.empty((6, len(self.thrusters)))
        for i, t in enumerate(self.thrusters):
            self.wrenchmat[0][i] = t.force_hat[0]
            self.wrenchmat[1][i] = t.force_hat[1]
            self.wrenchmat[2][i] = t.force_hat[2]
            self.wrenchmat[3][i] = t.torque[0]
            self.wrenchmat[4][i] = t.torque[1]
            self.wrenchmat[5][i] = t.torque[2]

        self.wrenchmat_pinv = np.linalg.pinv(self.wrenchmat) # Calculate pseudoinverse of wrench matrix


    def calc_thruster_allocs(self, pid_wrench):
        # pid_wrench = [x, y, z, roll, pitch, yaw] (PID control efforts)
        # Calculate thruster allocations using pseudoinverse of wrench matrix
        # TODO: Explain math in readme, not here
        thruster_allocations = np.matmul(self.wrenchmat_pinv, pid_wrench)

        # Scale all thruster allocations so no allocation has magnitude > 1
        max_pow = np.max(np.abs(thruster_allocations))
        if max_pow > 1:
            thruster_allocations /= max_pow

        # Round really small thruster outputs to 0
        thruster_allocations[np.abs(thruster_allocations) < 0.001] = 0

        return thruster_allocations
