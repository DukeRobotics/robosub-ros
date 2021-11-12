#!/usr/bin/env python3.9

import sim
from time import sleep


def run_sim_function(func, args):
    res = func(*args)
    if not isinstance(res, list) and not isinstance(res, tuple):
        res = (res,)
    if res[0] != sim.simx_return_ok and args[-1] != sim.simx_opmode_streaming:
        print(f'Error calling simulation. Code: {res[0]}')
    if len(res) == 1:
        return None
    if len(res) == 2:
        return res[1]
    return res[1:]

if __name__ == "__main__":
    pass

sim.simxFinish(-1)
clientID = sim.simxStart("127.0.0.1", 8000, True, True, 5000, 5)

if clientID == -1:
    print('Failed connecting to remote API server')
    sim.simxFinish(-1)
    exit(1)
print('Connected to remote API server')
print('Testing connection')
objs = run_sim_function(sim.simxGetObjects, (clientID, sim.sim_handle_all, sim.simx_opmode_blocking))
print(f'Number of objects in the scene: {len(objs)}')
robot = run_sim_function(sim.simxGetObjectHandle, (clientID, "Cuboid", sim.simx_opmode_blocking))

while True:
    run_sim_function(sim.simxCallScriptFunction, (clientID, "Cuboid", sim.sim_scripttype_childscript,
                                                       "addForceWrapper",
                                                       [robot], [0.0, 0.0, 9.82], [""], bytearray(),
                                                       sim.simx_opmode_blocking))
