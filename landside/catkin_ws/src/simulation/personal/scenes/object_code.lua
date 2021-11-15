require "math"
require "common_code"

function extsysCall_init()
    hr = sim.getObjectAssociatedWithScript(sim.handle_self)
    buoyancyEnabled = 1
    dragType = 1 -- 0 for linear, 1 for quadratic
    dragCoef = 1
    anchorPoints = {}
    initPos = sim.getObjectPosition(hr, -1)
    initQuat = sim.getObjectQuaternion(hr, -1)

    waterlevel = 0
    p = 1000
end

function extsysCall_actuation()
    if buoyancyEnabled ~= 0 then
        applyBuoyancy()
    end
end