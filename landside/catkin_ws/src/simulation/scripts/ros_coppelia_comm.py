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
sim.simxFinish(-1)  # just in case, close all opened connections
if len(sys.argv) == 1:
    ip = '127.0.0.1'
else:
    ip = sys.argv[1]
    print("Using ip address: " + ip)
clientID = sim.simxStart(ip, 8080, True, True, 5000, 5)  # For most computers
# clientID=sim.simxStart('192.168.99.100',8080,True,True,5000,5) # Use if you have Docker Toolbox (i.e. just Windows
# without full Docker)
winID = sim.simxStart('127.0.0.1', 5555, True, True, 5000, 5)
if clientID != -1 and winID != -1:
    print('Connected to remote API server')

    # Now try to retrieve data in a blocking fashion (i.e. a service call):
    res, objs = sim.simxGetObjects(clientID, sim.sim_handle_all, sim.simx_opmode_blocking)
    if res == sim.simx_return_ok:
        print('Number of objects in the scene: ', len(objs))
    else:
        print('Remote API function call returned with error code: ', res)

    time.sleep(2)

    # Now retrieve streaming data (i.e. in a non-blocking fashion):
    startTime = time.time()
    # sim.simxGetIntegerParameter(clientID,sim.sim_intparam_mouse_x,sim.simx_opmode_streaming) # Initialize streaming
    data = []
    ints = []
    strs = []
    while True:  # time.time()-startTime < 20:
        # Try to retrieve the streamed data
        # returnCode,data=sim.simxGetIntegerParameter(clientID,sim.sim_intparam_mouse_x,sim.simx_opmode_buffer)
        # After initialization of streaming, it will take a few ms before the first value arrives, so check return code
        # if returnCode==sim.simx_return_ok:
        #    print ('Mouse position x: ',data) # Mouse position x is actualized when the cursor is over V-REP's window
        res, ints, data, strs, byts = sim.simxCallScriptFunction(clientID, "Cuboid", sim.sim_scripttype_childscript,
                                                                 "get_ros_data",
                                                                 ints, data, strs, bytearray(),
                                                                 sim.simx_opmode_blocking)
        print("from docker sim", data)
        res, ints, data, strs, byts = sim.simxCallScriptFunction(winID, "Rob", sim.sim_scripttype_childscript,
                                                                 "read_ros_data",
                                                                 ints, data, strs, bytearray(),
                                                                 sim.simx_opmode_blocking)
        print("from robot sim", data)
        time.sleep(0.005)
else:
    print('Failed connecting to remote API server')
    print(clientID)
    print(winID)

    sim.simxFinish(-1)  # just in case, close all opened connections

print('Program ended')
