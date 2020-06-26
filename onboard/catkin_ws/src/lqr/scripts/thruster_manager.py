#!/usr/bin/env python

import yaml
import numpy as np
from thruster import Thruster


class ThrusterManager:

    def __init__(self, config_filename):
        # Load vehicle configuration file
        with open(config_filename) as f:
            self.vehicle = yaml.load(f)

        # Initialize thrusters based off of configuration file
        self.thrusters = []
        for t_dict in self.vehicle['thrusters']:
            t = Thruster(t_dict['pos'], t_dict['rpy'], t_dict['flipped'])
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

