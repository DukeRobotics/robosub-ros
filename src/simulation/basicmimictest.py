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
    import vrep
except Exception as e:
    print ('--------------------------------------------------------------')
    print ('"vrep.py" could not be imported. This means very probably that')
    print ('either "vrep.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "vrep.py"')
    print ('--------------------------------------------------------------')
    print ('')
    print (e)

import time

print ('Program started')
vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',8080,True,True,5000,5) # Connect to V-REP
winID=vrep.simxStart('127.0.0.1',20000,True,True,5000,5)
if clientID!=-1 and winID!=-1:
    print ('Connected to remote API server')

    # Now try to retrieve data in a blocking fashion (i.e. a service call):
    res,objs=vrep.simxGetObjects(clientID,vrep.sim_handle_all,vrep.simx_opmode_blocking)
    if res==vrep.simx_return_ok:
        print ('Number of objects in the scene: ',len(objs))
    else:
        print ('Remote API function call returned with error code: ',res)

    time.sleep(2)

    # Now retrieve streaming data (i.e. in a non-blocking fashion):
    startTime=time.time()
    #vrep.simxGetIntegerParameter(clientID,vrep.sim_intparam_mouse_x,vrep.simx_opmode_streaming) # Initialize streaming
    while time.time()-startTime < 20:
        #returnCode,data=vrep.simxGetIntegerParameter(clientID,vrep.sim_intparam_mouse_x,vrep.simx_opmode_buffer) # Try to retrieve the streamed data
        #if returnCode==vrep.simx_return_ok: # After initialization of streaming, it will take a few ms before the first value arrives, so check the return code
        #    print ('Mouse position x: ',data) # Mouse position x is actualized when the cursor is over V-REP's window
        res,objs=vrep.simxGetObjects(clientID,vrep.sim_handle_all,vrep.simx_opmode_streaming)
        res,wobjs=vrep.simxGetObjects(winID,vrep.sim_handle_all,vrep.simx_opmode_streaming)
        print (objs)
        print (wobjs)
        for i in range(len(objs)):
            if i >=len(wobjs):
                break
            res,pos = vrep.simxGetObjectPosition(clientID, objs[i],-1,vrep.simx_opmode_streaming)
            vrep.simxSetObjectPosition(winID, wobjs[i],-1,pos,vrep.simx_opmode_streaming)
            res,quat = vrep.simxGetObjectQuaternion(clientID, objs[i],-1,vrep.simx_opmode_streaming)
            vrep.simxSetObjectQuaternion(winID, wobjs[i],-1,quat,vrep.simx_opmode_streaming)
        time.sleep(0.005)

    # Now send some data to V-REP in a non-blocking fashion:
    vrep.simxAddStatusbarMessage(clientID,'Hello V-REP!',vrep.simx_opmode_oneshot)

    # Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    vrep.simxGetPingTime(clientID)

    # Now close the connection to V-REP:
    vrep.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')
print ('Program ended')
