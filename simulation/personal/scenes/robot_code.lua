require "math"

function extsysCall_init()
    --check to see relationship between inches and vrep units
    -- =~ 0.03
    a = 0.0254 --conversion from inches to vrep units
        --MC note: pretty sure "vrep units" are meters. Could be wrong, but adjusted a to match
    
    quadratic = true -- make true to set drag to quadratic
    
    --[[]]--Original values
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
    --]]
    
    --cx = 0.16971 -- percent needed if only front or back two bottom thrusters are used to straighten rob
    cx = 0.2136
    c2 = 0.08485 -- percent needed if all four bottom thrusters are used to straighten top thrusters at 1
    
    
    --Make forcesList a list of values, and the program will cycle through them, switching every 5 seconds
    forcesList = {{0,0,0,0,0,0,0,0}}--,
                        --{1,1,1,1,0,0,0,0},
                          --{-1,1,1,-1,0,0,0,0},
                          --{-1,-1,-1,-1,0,0,0,0},
                          --{1,-1,-1,1,0,0,0,0}}
    ct = 1
    
    values = forcesList[1]
    --values = {1,1,1,1,0,0,-cx,-cx}
    --values = {-1,1,-1,1,0,0,0,0}
    
    --values[5,6] = -0.01
        --all positive is forward and up
    
    simRemoteApi.start(20000)

    --order: top front right, left, top back right, left, bottom front right, left
    -- bottom back right, left
    hr=sim.getObjectAssociatedWithScript(sim.handle_self)
    top_front_right={dxtopfront,-dytop,dztop}
    top_front_left={dxtopfront,dytop,dztop}
    top_back_right={dxbacktop,-dytop,dztop}
    top_back_left={dxbacktop,dytop,dztop}
    bottom_front_right={dxbottomfront,-dybottom,dzbottom}
    bottom_front_left={dxbottomfront,dybottom,dzbottom}
    bottom_back_right={dxbackbottom,-dybottom,dzbottom}
    bottom_back_left={dxbackbottom,dybottom,dzbottom}
    
    --[[--Original Values
    top_front_right={dxtop,dytopfront,dztop}
    top_front_left={-dxtop,dytopfront,dztop}
    top_back_right={dxtop,dybacktop,dztop}
    top_back_left={-dxtop,dybacktop,dztop}
    bottom_front_right={dxbottom,dybottomfront,dzbottom}
    bottom_front_left={-dxbottom,dybottomfront,dzbottom}
    bottom_back_right={dxbottom,dybackbottom,dzbottom}
    bottom_back_left={-dxbottom,dybackbottom,dzbottom}
    --]]
    thrusterPoints={top_front_right,top_front_left,top_back_right,top_back_left,
                    bottom_front_right,bottom_front_left,bottom_back_right,bottom_back_left}
    numThrusters=8
    forces={}
    
    --from buoyancytest
    m=sim.getShapeMassAndInertia(hr)
    --print(3)
    waterlevel = 0
    p = 1000
    thrusterforce = 5.25 --4.1 rev
    thrusterforceforward = 5.25
    thrusterforcebackward = 4.1
    actualmass = 22
    fudgeforce = thrusterforce*m/actualmass
    dragcoef = 1.1 --original: 1.1
    globforce = {0,0,0}
    
    -- -4, 1, 3 from vincent
    --these numbers calc'd for a 15 deg pitch, and arbitrary roll (it looks about right)
    centerOfBuoy = {-.1749*a, -.15*a * 0, 2*a}
    --centerOfBuoy = {0,0,a}
    centerOfMass = {.032,  0, .005} --currently unused

    startTime = sim.getSystemTimeInMs(-1)
    lastTime = sim.getSystemTimeInMs(startTime)
    lastVel, lastAngVel = sim.getObjectVelocity(hr)
end

function extsysCall_actuation()
    pos = sim.getObjectPosition(hr, -1)
    --print("pos "..pos[3])
    res, xsizemin = sim.getObjectFloatParameter(hr, 15)
    res, ysizemin = sim.getObjectFloatParameter(hr, 16)
    res, zsizemin = sim.getObjectFloatParameter(hr, 17)
    res, xsizemax = sim.getObjectFloatParameter(hr, 18)
    res, ysizemax = sim.getObjectFloatParameter(hr, 19)
    res, zsizemax = sim.getObjectFloatParameter(hr, 20)
    xsize = xsizemax-xsizemin
    ysize = ysizemax-ysizemin
    zsize = zsizemax-zsizemin
    --print("zsize "..zsize)
    grav=sim.getArrayParameter(sim.arrayparam_gravity)
    fbuoy = xsize*ysize
    pos[3] = pos[3] - zsize/2 --fudge due to inconsistency with relative measurements (?)
    if zsize<=(waterlevel-pos[3]) then
        subdepth = zsize
    else
        subdepth = (waterlevel-pos[3])
        --print('hi')
    end
    fbuoy = fbuoy*subdepth*(-1)*grav[3]*p
    if pos[3]>waterlevel then 
        fbuoy = 0 
        subdepth = 0
    end
    
    v, angv =sim.getVelocity(hr)
    --print("angular velocity" .. angv[1])
    dragforcelin={0,0,0}
    dragforceang={0,0,0} --[1] roll, [2] pitch, [3] yaw

    
    --print(pos[3], fbuoy, globforce, dragforce, grav[3]*m)
    dragforcelin[1] = calc_dragforcelin(v[1],ysize,subdepth)
    dragforcelin[2] = calc_dragforcelin(v[2],xsize,subdepth)
    dragforcelin[3] = calc_dragforcelin(v[3],xsize,ysize)
    --if pos[3]>(waterlevel-zsize) and v[3]>0 then dragforcelin[3]=0 end
    if pos[3]>waterlevel then dragforcelin[3]=0 end

    dragforceang[1] = calc_dragforceang(angv[1], ysize, xsize)
    dragforceang[2] = calc_dragforceang(angv[2], ysize, xsize)
    dragforceang[3] = calc_dragforceang(angv[3], ysize, subdepth)

    --print("ang drag" .. dragforceang[1])
    lastTime = sim.getSystemTimeInMs(startTime)
    --print(lastTime .. " " .. startTime)
    if lastTime> 5000 then
        print("switch!")
        ct = (ct)%#forcesList+1
        values = forcesList[ct]
        print(values)
        startTime = sim.getSystemTimeInMs(-1)
    end

    sim.addForceAndTorque(hr, {dragforcelin[1],
                              dragforcelin[2],
                              dragforcelin[3]},dragforceang)--centerOfMass)
    --sim.addForceAndTorque(hr, {0,0,0}, dragforceang)
    transform = sim.getObjectMatrix(hr, -1)
    transform1 = sim.buildMatrix({0,0,0},sim.getObjectOrientation(hr, -1))
    transtransform = sim.invertMatrix(transform)
    --transtransform1 = sim.invertMatrix(transform)
    relbuoy = sim.multiplyVector(transform, {0,0,fbuoy})
    relbuoy1 = sim.multiplyVector(transform1, {0,0,fbuoy})
    --print("values")
    --print(relbuoy)
    --print(relbuoy1)
    sim.addForce(hr, centerOfBuoy, relbuoy)
    setForces(values)
    for i=1,numThrusters do
        sim.addForce(hr,thrusterPoints[i],forces[i])
    end
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
        return -p*math.abs(linvel^2)*get_sign(linvel)*dragcoef*length*depth
    end
    return -p*math.abs(linvel)*get_sign(linvel)*dragcoef*length*depth
end

function calc_dragforceang(angvel, length, depth)
    --if quadratic:
    -- -p * angvelocity * angvelocity * x * y * y * y * dragcoef / 12
    -- if linear
    -- -p * angvelocity * x * y * y * dragcoef / 4
    angdragfudgecoef = 1 -- 0.05
    if quadratic then
        return -p*math.abs(angvel^2) * get_sign(angvel) * dragcoef * length^3 * depth / 12 * angdragfudgecoef
    end
    return -p*math.abs(angvel^1) * get_sign(angvel) * dragcoef * length^2 * depth / 4 * angdragfudgecoef
    
end

function read_ros_data(inInts, inFloats, inString, inBuffer)
    print(inFloats)
    if (inFloats) then
        values = inFloats
        forcesList[1] = values
    end
    linvel, angvel = sim.getObjectVelocity(hr)
    quat = sim.getObjectQuaternion(hr, -1)
    pos = sim.getObjectPosition(hr, -1)
    accel = {}
    currentTime = sim.getSystemTimeInMs(startTime)
    dt = (currentTime-lastTime)/1000
    for i=1,3 do
        accel[i] = (linvel[i]-lastVel[i])/dt
    end
    lastTime = currentTime
    lastVel = linvel
    outFloats = {}
    feeder = {linvel, angvel, quat, accel}
    trans = sim.buildMatrix({0,0,0}, sim.getObjectOrientation(hr, -1))
    res = sim.invertMatrix(trans)
    linvel = sim.multiplyVector(trans, linvel)
    --[[for i = 1, 12 do
        outFloats[i] = feeder[math.floor((i-1)/3)+1][i%3]
    end ]]-- Loop to do the following 12 lines of code, currently not working
    outFloats[1] = linvel[1]
    outFloats[2] = linvel[2]
    outFloats[3] = linvel[3]
    outFloats[4] = angvel[1]
    outFloats[5] = angvel[2]
    outFloats[6] = angvel[3]
    outFloats[7] = quat[1]
    outFloats[8] = quat[2]
    outFloats[9] = quat[3]
    outFloats[10] = quat[4]
    outFloats[11] = accel[1]
    outFloats[12] = accel[2]
    outFloats[13] = accel[3]
    outFloats[14] = pos[1]
    outFloats[15] = pos[2]
    outFloats[16] = pos[3]
    --outFloats[17] = currentTime/1000
    --print(outFloats)
    return {}, outFloats, {}, ""
end

function setForces(vals)
    if (vals == nil) then
        return
    end
    --print(vals)
    --[[]]
    forces[1]={1,1,0} --tfr
    forces[2]={1,-1,0} --tfl
    forces[3]={1,-1,0} --tbr
    forces[4]={1,1,0} --tbl
    forces[5]={0,0,1}
    forces[6]={0,0,1} 
    forces[7]={0,0,1}
    forces[8]={0,0,1}
    --]]
    --[[--Original Values
    forces[1]={-1,1,0} --tfr
    forces[2]={1,1,0} --tfl
    forces[3]={1,1,0} --tbr
    forces[4]={-1,1,0} --tbl
    forces[5]={0,0,1}
    forces[6]={0,0,1} 
    forces[7]={0,0,1}
    forces[8]={0,0,1}
    --]]

    for count = 1,numThrusters do
        for num =1, 3 do
            if count < 5 then 
                forces[count][num] = (vals[count])*((1/math.sqrt(2))*fudgeforce*(forces[count][num]))                            
            else
                forces[count][num] = (vals[count])*fudgeforce*(forces[count][num])
            end
        end
    end
end
