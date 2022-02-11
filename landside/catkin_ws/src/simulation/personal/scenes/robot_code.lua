require "math"

function extsysCall_init()
  simRemoteApi.start(5555, 1300, true, false)
    --check to see relationship between inches and vrep units
    -- =~ 0.03
    a = 0.0254 --conversion from inches to vrep units
    --MC note: pretty sure "vrep units" are meters. Could be wrong, but adjusted a to match

    quadratic = true -- make true to set drag to quadratic

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
    waterlevel = 0
    p = 1000
    dragcoef = 1.1 --original: 1.1 , try 0.9 next
    --too low makes it not move down bc robot is busy fixing angular position
    --too high makes it not move down bc robot can't overcome drag
    --0.7-1.1 seems to be sweet spot, but even that's not good enough -- robot does not continue moving down
    --seems drag constant might be dependent on velocity (bc it's dependent on Reynold's number)

    -- -4, 1, 3 from vincent
    --these numbers calc'd for a 15 deg pitch, and arbitrary roll (it looks about right)
    --centerOfBuoy = {-.1749*a, -.15*a * 0, 2*a}
    centerOfBuoy = { 0, 0, 2 * a }
    centerOfMass = { .032, 0, .005 } --currently unused
end

function extsysCall_actuation()
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

    grav = sim.getArrayParameter(sim.arrayparam_gravity) -- gravitational acceleration
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
    relbuoy_mag = math.sqrt(relbuoy[1]^2 + relbuoy[2]^2 + relbuoy[3]^2)
    relbuoy_normalized = {relbuoy[1]/relbuoy_mag*fbuoy, relbuoy[2]/relbuoy_mag*fbuoy, relbuoy[3]/relbuoy_mag*fbuoy}

    v, angv = sim.getVelocity(hr)
    dragforcelin = {calc_dragforcelin(v[1], ysize, subdepth), -- linear drag force
                    calc_dragforcelin(v[2], xsize, subdepth),
                    calc_dragforcelin(v[3], xsize, ysize)}
    if pos[3] > waterlevel then
        dragforcelin[3] = 0
    end
    dragforceang = {calc_dragforceang(angv[1], ysize, xsize), -- angular drag force
                    calc_dragforceang(angv[2], ysize, xsize),
                    calc_dragforceang(angv[3], ysize, subdepth)}

    sim.addForceAndTorque(hr, dragforcelin, dragforceang)
    sim.addForce(hr, centerOfBuoy, relbuoy_normalized) -- buoyancy force
    for i = 1, table.getn(forces) do
        sim.addForce(hr, thrusterPoints[i], forces[i]) -- thruster force
    end
    
    if m ~= nil and forces[5] ~= nil then
    print(
        "Drag: ", dragforcelin[3],"\n",
        "Rel Buoyancy Normalized: ", relbuoy_normalized, "\n", --could try just fbuoy, relbuoy is fbuoy after transform
        "Rel Buoyancy Norm: ", math.sqrt(relbuoy_normalized[1]^2 + relbuoy_normalized[2]^2 + relbuoy_normalized[3]^2), "\n",
        "Buoyancy: ", fbuoy, "\n",
        "Gravity: ", grav[3] * m, "\n",
        "Thruster force: ", forces[5][3], forces[6][3], forces[7][3], forces[8][3], "\n",
        "Total Force: ", dragforcelin[3] + relbuoy[3] + grav[3] * m + forces[5][3] + forces[6][3] + forces[7][3]+ forces[8][3], "\n"
    ) end
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
    if quadratic then
        return -p * math.abs(linvel ^ 2) * get_sign(linvel) * dragcoef * length * depth
        
    end
    return -p * math.abs(linvel) * get_sign(linvel) * dragcoef * length * depth
    --return -6 * math.pi * 0.00105 * (length/2) * math.abs(linvel ^ 2) * get_sign(linvel)
end

function calc_dragforceang(angvel, length, depth)
    --if quadratic:
    -- -p * angvelocity * angvelocity * x * y * y * y * dragcoef / 12
    -- if linear
    -- -p * angvelocity * x * y * y * dragcoef / 4
    
    angdragfudgecoef = 1 -- 0.05
    if quadratic then
        return -p * math.abs(angvel ^ 2) * get_sign(angvel) * dragcoef * length ^ 3 * depth / 12 * angdragfudgecoef
    end
    return -p * math.abs(angvel ^ 1) * get_sign(angvel) * dragcoef * length ^ 2 * depth / 4 * angdragfudgecoef
    --return -6 * math.pi * 0.00105 * (length/2)^3 * math.abs(angvel ^ 2) * get_sign(angvel)

end

function setThrusterForces(inInts, inFloats, inString, inBuffer)
  for i = 1,8 do
    forces[i] = {inFloats[(i-1)*3 + 1],
                 inFloats[(i-1)*3 + 2],
                 inFloats[(i-1)*3 + 3]}
  end
  return {},{},{},''
end

function getMass(intParams,floatParams,stringParams,bufferParam)
  m = sim.getShapeMassAndInertia(intParams[1])
  return {},{m},{},''
end
