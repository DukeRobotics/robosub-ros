require "math"
require "common_code"

function extsysCall_init()
  simRemoteApi.start(5555, 1300, true, false)
    --check to see relationship between inches and vrep units
    -- =~ 0.03
    a = 0.0254 --conversion from inches to vrep units
    --MC note: pretty sure "vrep units" are meters. Could be wrong, but adjusted a to match

    -- change this for new robot
    dytop = 7.69 * a--same front and back
    dybottom = 6.47 * a
    dxtopfront = 9.71 * a
    dxbottomfront = 9.75 * a

    dxbacktop = -7.69 * a--assume top is same as bottom
    dxbackbottom = -7.75 * a
    dztop = 1.17 * a
    --MC note: for stability. Replace with commented values for more accuracy
    --MC note: density assumed to be uniform? In any case, very little force rotates the bot

    dzbottom = -5.44 * a

    --order: top front right, left, top back right, left, bottom front right, left
    -- bottom back right, left
    hr = sim.getObjectAssociatedWithScript(sim.handle_self)
    top_front_right = { dxtopfront, -dytop, dztop }
    top_front_left = { dxtopfront, dytop, dztop }
    top_back_right = { dxbacktop, -dytop, dztop }
    top_back_left = { dxbacktop, dytop, dztop }
    bottom_front_right = { dxbottomfront, -dybottom, dzbottom }
    bottom_front_left = { dxbottomfront, dybottom, dzbottom }
    bottom_back_right = { dxbackbottom, -dybottom, dzbottom }
    bottom_back_left = { dxbackbottom, dybottom, dzbottom }

    thrusterPoints = { top_front_right, top_front_left, top_back_right, top_back_left,
                       bottom_front_right, bottom_front_left, bottom_back_right, bottom_back_left }

    forces = {}
    buoyancyEnabled = 1
    dragType = 1 -- 0 for linear, 1 for quadratic
    dragCoef = 1.1
    angdragcoefroll = 1 --0.1 to do barrel roll
    angdragcoefpitch = 6 --0.1 to do flips
    angdragcoefyaw = 1 --0.1 to do rotations about z axis

    anchorPoints = {}
    initPos = sim.getObjectPosition(hr, -1)
    initQuat = sim.getObjectQuaternion(hr, -1)

    waterlevel = 0
    p = 1000

    -- -4, 1, 3 from vincent
    --these numbers calc'd for a 15 deg pitch, and arbitrary roll (it looks about right)
    --centerOfBuoy = {-.1749*a, -.15*a * 0, 2*a}
    centerOfBuoy = { 0, 0, 2 * a }
    centerOfMass = { .032, 0, .005 } --currently unused
end

function extsysCall_actuation()
    if buoyancyEnabled ~= 0 then
        applyBuoyancy()
    end
    for i = 1, table.getn(forces) do
        sim.addForce(hr, thrusterPoints[i], forces[i])
    end
end

function setThrusterForces(inInts, inFloats, inString, inBuffer)
  for i = 1,8 do
    forces[i] = {inFloats[(i-1)*3 + 1],
                 inFloats[(i-1)*3 + 2],
                 inFloats[(i-1)*3 + 3]}
  end
  return {},{},{},''
end