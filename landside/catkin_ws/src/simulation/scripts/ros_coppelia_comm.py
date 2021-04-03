# Make sure to have the server side running in V-REP:
# in a child script of a V-REP scene, add following command
# to be executed just once, at simulation start:
#
# simRemoteApi.start(19999)
#
# then start simulation, and run this program.
#
# IMPORTANT: for each successful call to simxStart, there
# should be a corresponding call to simxFinish at the end!

try:
    import sim
except Exception as e:
    print('--------------------------------------------------------------')
    print('"sim.py" could not be imported. This means very probably that')
    print('either "sim.py" or the remoteApi library could not be found.')
    print('Make sure both are in the same folder as this file,')
    print('or appropriately adjust the file "sim.py"')
    print('--------------------------------------------------------------')
    print('')
    print(e)

import sys
import time

print('Program started')

if len(sys.argv) != 2:
    print("No IP address provided")
    sys.exit(1)
sim.simxFinish(-1)  # just in case, close all opened connections
clientID = sim.simxStart(sys.argv[1], 8080, True, True, 5000, 5)

if clientID == -1:
    print('Failed connecting to remote API server')
    sim.simxFinish(-1)
    sys.exit(1)

print('Connected to remote API server')

# Now try to retrieve data in a blocking fashion (i.e. a service call):
res, objs = sim.simxGetObjects(clientID, sim.sim_handle_all, sim.simx_opmode_blocking)
if res == sim.simx_return_ok:
    print('Number of objects in the scene: ', len(objs))
else:
    print('Remote API function call returned with error code: ', res)

res, handle = sim.simxGetObjectHandle(clientID, "Rob", sim.simx_opmode_blocking)
res = sim.simxSetObjectPosition(clientID, handle, -1, [0.0, 0.0, 0.0], sim.simx_opmode_oneshot)

while True:
    res, ints, data, strs, byts = sim.simxCallScriptFunction(clientID, "Rob", sim.sim_scripttype_childscript,
                                                                    "addForceAndTorque_function",
                                                                    [handle], [1.0, 1.0, 1.0, 1.0, 1.0, 1.0], [""], bytearray(),
                                                                    sim.simx_opmode_blocking)

sim.simxFinish(-1)