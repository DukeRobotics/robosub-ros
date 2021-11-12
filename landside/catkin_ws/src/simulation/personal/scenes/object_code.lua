require "math"

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

function reset(inInts, inFloats, inString, inBuffer)
    sim.setObjectPosition(hr, -1, initPos)
    sim.setObjectQuaternion(hr, -1, initPos)
    return {}, {}, {}, ''
end

function getName(inInts, inFloats, inString, inBuffer)
    return {}, {}, {sim.getObjectName(hr)}, ''
end

function extsysCall_actuation()
    if buoyancyEnabled ~= 0 then
        applyBuoyancy()
    end
end

function applyBuoyancy()
    pos = sim.getObjectPosition(hr, -1)

    res, xsizemin = sim.getObjectFloatParameter(hr, 15)
    res, ysizemin = sim.getObjectFloatParameter(hr, 16)
    res, zsizemin = sim.getObjectFloatParameter(hr, 17)
    res, xsizemax = sim.getObjectFloatParameter(hr, 18)
    res, ysizemax = sim.getObjectFloatParameter(hr, 19)
    res, zsizemax = sim.getObjectFloatParameter(hr, 20)
    xsize = xsizemax - xsizemin
    ysize = ysizemax - ysizemin
    zsize = zsizemax - zsizemin

    grav = sim.getArrayParameter(sim.arrayparam_gravity)
    pos[3] = pos[3] - zsize / 2 --fudge due to inconsistency with relative measurements (?)
    if zsize <= (waterlevel - pos[3]) then
        subdepth = zsize
    else
        subdepth = (waterlevel - pos[3])
    end
    fbuoy = xsize * ysize * subdepth * (-1) * grav[3] * p
    if pos[3] > waterlevel then
        fbuoy = 0
        subdepth = 0
    end

    transform = sim.getObjectMatrix(hr, -1)
    res = sim.invertMatrix(transform)
    relbuoy = sim.multiplyVector(transform, { 0, 0, fbuoy })

    v, angv = sim.getVelocity(hr)
    dragforcelin = {calc_dragforcelin(v[1], ysize, subdepth),
                    calc_dragforcelin(v[2], xsize, subdepth),
                    calc_dragforcelin(v[3], xsize, ysize)}
    if pos[3] > waterlevel then
        dragforcelin[3] = 0
    end
    dragforceang = {calc_dragforceang(angv[1], ysize, xsize),
                    calc_dragforceang(angv[2], ysize, xsize),
                    calc_dragforceang(angv[3], ysize, subdepth)}

    sim.addForceAndTorque(hr, dragforcelin, dragforceang)
    sim.addForce(hr, {0,0,0}, relbuoy) -- 0,0,0 is center of buoyancy, here
end

function get_sign(x)
    if (x > 0) then
        return 1
    elseif (x < 0) then
        return -1
    else
        return 0
    end
end

function calc_dragforcelin(linvel, length, depth)
    if dragType ~= 0 then
        return -p * math.abs(linvel ^ 2) * get_sign(linvel) * dragCoef * length * depth
    end
    return -p * math.abs(linvel) * get_sign(linvel) * dragCoef * length * depth
end

function calc_dragforceang(angvel, length, depth)
    --if quadratic:
    -- -p * angvelocity * angvelocity * x * y * y * y * dragCoef / 12
    -- if linear
    -- -p * angvelocity * x * y * y * dragCoef / 4
    angdragfudgecoef = 1 -- 0.05
    if dragType ~= 0 then
        return -p * math.abs(angvel ^ 2) * get_sign(angvel) * dragCoef * length ^ 3 * depth / 12 * angdragfudgecoef
    end
    return -p * math.abs(angvel ^ 1) * get_sign(angvel) * dragCoef * length ^ 2 * depth / 4 * angdragfudgecoef

end

-- Enables or disables simulation of drag and buoyancy for this object.
-- Params:
--   inInts: Array whose first value is nonzero if buoyancy should be enabled
--   and 0 otherwise
-- Outputs:
--   None
function enableBuoyancyDrag(inInts, inFloats, inString, inBuffer)
    buoyancyEnabled = inInts[1]
    return {},{},{},''
end

-- Sets the drag coefficient for this object.
-- Params:
--   inFloats: Array whose first value is the new drag coefficient.
-- Outputs:
--   None
function setDragCoefficient(inInts, inFloats, inString, inBuffer)
    dragCoef = inFloats[1]
    return {},{},{},''
end

-- Sets the type of drag for this object to linear or quadratic.
-- Params:
--   inInts: Array whose first value 0 if linear and nonzero
--   if quadratic
-- Outputs:
--   None
function setDragType(inInts, inFloats, inString, inBuffer)
    dragType = inInts[1]
    return {},{},{},''
end

-- Sets the mass of this object.
-- Params:
--   inFloats: Array whose first value is the mass of the object.
-- Outputs:
--   None
function setMass(inInts, inFloats, inString, inBuffer)
    sim.setShapeMass(handle, inFloats[1])
    return {},{},{},''
end

function getHandle(inInts, inFloats, inString, inBuffer)
    return {hr},{},{},''
end

-- Adds an anchor point to this object. An anchor point represents a rope
-- which is tied to the object at one end and tied to the floor at the other.
-- Params:
--   TODO: Decide params
-- Outputs:
--   None
function addAnchorPoint(inInts, inFloats, inString, inBuffer)
    return {},{},{},''
end

-- Removes an anchor point from this object. An anchor point represents a rope
-- which is tied to the object at one end and tied to the floor at the other.
-- Params:
--   TODO: Decide params
-- Outputs:
--   None
function removeAnchorPoint(inInts, inFloats, inString, inBuffer)
    return {},{},{},''
end