require "math"


function addForceAndTorque_function(intParams,floatParams,stringParams,bufferParam)
    objectHandleT = intParams
    force_torqueT = floatParams
    return {},{},{},''
end

function addForce_function(intParams,floatParams,stringParams,bufferParam)
    objectHandleF = intParams
    forceF = floatParams
    return {},{},{},''
end

function extsysCall_init()
    simRemoteApi.start(8080)
    objectHandleT = {}
    force_torqueT = {}
    objectHandleF = {}
    forceF = {}
end

function extsysCall_actuation()
    for i=1,table.getn(objectHandleT) do
        sim.addForceAndTorque(objectHandleT[i], 
                            { force_torqueT[(i-1)*6 + 1], 
                              force_torqueT[(i-1)*6 + 2], 
                              force_torqueT[(i-1)*6 + 3] },
                            { force_torqueT[(i-1)*6 + 4],
                              force_torqueT[(i-1)*6 + 5],
                              force_torqueT[(i-1)*6 + 6] })
    end

    for i=1,table.getn(objectHandleF) do
        sim.addForce(objectHandleF[i], 
                    { forceF[(i-1)*6 + 1], 
                      forceF[(i-1)*6 + 2], 
                      forceF[(i-1)*6 + 3] },
                    { forceF[(i-1)*6 + 4],
                      forceF[(i-1)*6 + 5],
                      forceF[(i-1)*6 + 6] })
    end
    
end
