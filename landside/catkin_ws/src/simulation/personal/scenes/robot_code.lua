require "math"
socket = require("socket")

function addDragForce(intParams,floatParams,stringParams,bufferParam)
    objectHandleT = intParams
    forceTorqueT = floatParams
    return {},{},{},''
end

function addBuoyancyForce(intParams,floatParams,stringParams,bufferParam)
    objectHandleBuoyF = intParams
    forceBuoyF = floatParams
    return {},{},{},''
end

function addThrusterForce(intParams,floatParams,stringParams,bufferParam)
  objectHandleThrusterF = intParams
  forceThrusterF = floatParams
  return {},{},{},''
end

function getMass(intParams,floatParams,stringParams,bufferParam)
  m = sim.getShapeMassAndInertia(intParams[1])
  return {},{m},{},''
end

function extsysCall_init()
    simRemoteApi.start(8080)
    objectHandleT = {}
    forceTorqueT = {}
    objectHandleBuoyF = {}
    forceBuoyF = {}
    objectHandleThrusterF = {}
    forceThrusterF = {}
end

function extsysCall_actuation()

    for i=1,table.getn(objectHandleT) do
        sim.addForceAndTorque(objectHandleT[i], 
                            { forceTorqueT[(i-1)*6 + 1], 
                              forceTorqueT[(i-1)*6 + 2], 
                              forceTorqueT[(i-1)*6 + 3] },
                            { forceTorqueT[(i-1)*6 + 4],
                              forceTorqueT[(i-1)*6 + 5],
                              forceTorqueT[(i-1)*6 + 6] })
    end

    for i=1,table.getn(objectHandleBuoyF) do
        sim.addForce(objectHandleBuoyF[i], 
                    { forceBuoyF[(i-1)*6 + 1], 
                      forceBuoyF[(i-1)*6 + 2], 
                      forceBuoyF[(i-1)*6 + 3] },
                    { forceBuoyF[(i-1)*6 + 4],
                      forceBuoyF[(i-1)*6 + 5],
                      forceBuoyF[(i-1)*6 + 6] })
    end
    
    for i=1,table.getn(objectHandleThrusterF) do
      sim.addForce(objectHandleThrusterF[i], 
                  { forceThrusterF[(i-1)*6 + 1], 
                    forceThrusterF[(i-1)*6 + 2], 
                    forceThrusterF[(i-1)*6 + 3] },
                  { forceThrusterF[(i-1)*6 + 4],
                    forceThrusterF[(i-1)*6 + 5],
                    forceThrusterF[(i-1)*6 + 6] })
  end

end
