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
        
        self.wrenchmat = np.empty(6, len(thrusters))
        for i, t in enumerate(thrusters):
            self.wrenchmat[i][0] = t.force_hat.x
            self.wrenchmat[i][1] = t.force_hat.y
            self.wrenchmat[i][2] = t.force_hat.z
            self.wrenchmat[i][3] = t.torque.x
            self.wrenchmat[i][4] = t.torque.y
            self.wrenchmat[i][5] = t.torque.z

        self.wrenchmat_pinv = np.linalg.pinv(self.wrenchmat)


    def subscribe_to_pid():
        # TODO: actually subscribe to PID messages from ROS
        pid = np.random.rand(6)

        thruster_allocations = self.wrenchmat_pinv * pid

        # TODO: publish these bad boys
