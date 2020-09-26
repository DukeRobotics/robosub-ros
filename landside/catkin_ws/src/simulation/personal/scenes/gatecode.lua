function extsysCall_init()
    -- do some initialization here
    lc = sim.getObjectHandle("GateLeftChild")
    rc = sim.getObjectHandle("GateRightChild")

    hr = sim.getObjectHandle("Rob")

    rel_cam_pos = { 0.309, 0.138, 0.18 }   --relative position of camera
    robScriptHandle = sim.getScriptHandle("Rob")
end

function extsysCall_actuation()
    -- put your actuation code here
end

function get_GridPoint(gate_pt)
    orientation = sim.getObjectOrientation(hr, -1)
    robPos = sim.getObjectPosition(hr, -1)
    --print(orientation)
    rob_trans_matrix = sim.buildMatrix(robPos, orientation)
    trans_matrix_inv = sim.invertMatrix(rob_trans_matrix)
    --abs_cam_pos = sim.multiplyVector(trans_matrix_inv, rel_cam_pos)  --absolute camera position
    --rob_trans_matrix = sim.getObjectMatrix(hr, -1)

    -- supposed to be same point as gate_pt but with reference of robot
    gate_pt_new = sim.multiplyVector(rob_trans_matrix, gate_pt)
    gate_pt_reltocam = { gate_pt_new[1] - rel_cam_pos[1], gate_pt_new[2] - rel_cam_pos[2], gate_pt_new[3] - rel_cam_pos[3] }
    robToGate = { gate_pt[1] - robPos[1], gate_pt[2] - robPos[2], gate_pt[3] - robPos[3] }
    --gate_pt_new = sim.multiplyVector(rob_trans_matrix, robToGate)
    xFOV = 0.933 * gate_pt_reltocam[1]
    yFOV = 0.586 * gate_pt_reltocam[1]
    xPix = (xFOV / 2 - gate_pt_reltocam[2]) / xFOV
    yPix = (yFOV / 2 - gate_pt_reltocam[3]) / yFOV
    gridPt = { xPix, yPix }
    --print(gate_pt_new)
    --[[
    mag1 = calc_magnitude(gate_pt)
    mag2 = calc_magnitude(gate_pt_new)
    mag3 = calc_magnitude(robToGate)
    print("orig " ..mag1)
    print("new " ..mag2)
    print("should be same "..mag3)
--]]
    --maybe change to -1
    for i = 1, 2 do
        if (gridPt[i] < 0) then
            gridPt[i] = 0
        end
        if (gridPt[i] > 1) then
            gridPt[i] = 1
        end
    end
    return gridPt
end

function get_BoundingBox()
    leftGatePos = sim.getObjectPosition(lc, -1)
    rightGatePos = sim.getObjectPosition(rc, -1)
    res, leftzmax = sim.getObjectFloatParameter(lc, 20)
    res, rightzmax = sim.getObjectFloatParameter(rc, 20)
    res, leftzmin = sim.getObjectFloatParameter(lc, 17)
    res, rightzmin = sim.getObjectFloatParameter(rc, 17)

    topLeft = { leftGatePos[1], leftGatePos[2], leftGatePos[3] + leftzmax }
    bottomLeft = { leftGatePos[1], leftGatePos[2], leftGatePos[3] + leftzmin }
    topRight = { rightGatePos[1], rightGatePos[2], rightGatePos[3] + rightzmax }
    bottomRight = { rightGatePos[1], rightGatePos[2], rightGatePos[3] + rightzmin }
    fourGlobalPoints = { topLeft, topRight, bottomLeft, bottomRight }
    boundingBoxPoints = {}
    xs = {}
    ys = {}

    for i = 1, table.getn(fourGlobalPoints) do
        boundingBoxPoints[i] = get_GridPoint(fourGlobalPoints[i])
    end
    for i = 1, table.getn(boundingBoxPoints) do
        xs[i] = boundingBoxPoints[i][1]
        ys[i] = boundingBoxPoints[i][2]
    end
    xmax = 0
    xmin = 1
    ymax = 0
    ymin = 1
    for i = 1, table.getn(xs) do
        if (xs[i] > xmax) then
            xmax = xs[i]
        end
        if (xs[i] < xmin) then
            xmin = xs[i]
        end
        if (ys[i] > ymax) then
            ymax = ys[i]
        end
        if (ys[i] < ymin) then
            ymin = ys[i]
        end
    end
    return { xmin, xmax, ymin, ymax }
end

function sysCall_sensing()
    -- put your sensing code here
end

function sysCall_cleanup()
    -- do some clean-up here
end

-- See the user manual or the available code snippets for additional callback functions and details
