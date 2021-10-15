require "math"

function extsysCall_init()
end

function extsysCall_actuation()
end

-- Enables or disables simulation of drag and buoyancy for this object.
-- Params:
--   inInts: Array whose first value is nonzero if buoyancy should be enabled
--   and 0 otherwise
-- Outputs:
--   None
function applyBuoyancyDrag(inInts, inFloats, inString, inBuffer)
    return {},{},{},''
end

-- Sets the drag coefficient for this object.
-- Params:
--   inFloats: Array whose first value is the new drag coefficient.
-- Outputs:
--   None
function setDragCoefficient(inInts, inFloats, inString, inBuffer)
    return {},{},{},''
end

-- Sets the type of drag for this object to linear or quadratic.
-- Params:
--   inInts: Array whose first value 0 if linear and greater than 0 
--   if quadratic
-- Outputs:
--   None
function setDragType(inInts, inFloats, inString, inBuffer)
    return {},{},{},''
end

-- Sets the mass of this object.
-- Params:
--   inFloats: Array whose first value is the mass of the object.
-- Outputs:
--   None
function setMass(inInts, inFloats, inString, inBuffer)
    return {},{},{},''
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
