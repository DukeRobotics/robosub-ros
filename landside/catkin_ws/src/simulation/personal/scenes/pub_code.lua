require 'math'

function tableConcat(t1, t2)
    for i = 1, #t2 do
        t1[#t1 + 1] = t2[i]
    end
    return t1
end

function ros_publishing(inInts, inFloats, inString, inBuffer)
    HEAD_FLAG = -525600
    ARRAY_FLAG = -8675309
    linvel, angvel = sim.getObjectVelocity(hr)
    quat = sim.getObjectQuaternion(hr, -1)
    pos = sim.getObjectPosition(hr, -1)
    accel = {}
    currentTime = sim.getSystemTimeInMs(startTime)
    dt = (currentTime - lastTime) / 1000
    for i = 1, 3 do
        accel[i] = (linvel[i] - lastVel[i]) / dt
    end
    lastTime = currentTime
    lastVel = linvel
    outFloats = {}
    feeder = { linvel, angvel, quat, accel }
    trans = sim.buildMatrix({ 0, 0, 0 }, sim.getObjectOrientation(hr, -1))
    res = sim.invertMatrix(trans)
    linvel = sim.multiplyVector(trans, linvel)

    outDvl = { '/sim/dvl', 'geometry_msgs/TwistStamped', 'header',
               "twist.linear.x", "twist.linear.y", "twist.linear.z",
               "twist.angular.x", "twist.angular.y", "twist.angular.z" }

    outDvlData = { -1, -1, HEAD_FLAG }
    outDvlData = tableConcat(outDvlData, linvel)
    outDvlData = tableConcat(outDvlData, angvel)

    outPose = { '/sim/pose', 'geometry_msgs/PoseStamped', 'header',
                "pose.position.x", "pose.position.y", "pose.position.z",
                "pose.orientation.x", "pose.orientation.y", "pose.orientation.z", "pose.orientation.w" }

    outPoseData = { -1, -1, HEAD_FLAG }
    outPoseData = tableConcat(outPoseData, pos)
    outPoseData = tableConcat(outPoseData, quat)

    outArray = { '/sim/test_array', 'std_msgs/Float32MultiArray', 'data:14,13,151,12' }
    outArrayData = { -1, -1, ARRAY_FLAG }

    outImu = { '/sim/imu', 'sensor_msgs/Imu', "header",
               "orientation.x", "orientation.y", "orientation.z", "orientation.w",
               "orientation_covariance:0,0,0,0,0,0,0,0,0",
               "angular_velocity.x", "angular_velocity.y", "angular_velocity.z",
               "angular_velocity_covariance:0,0,0,0,0,0,0,0,0",
               "linear_acceleration.x", "linear_acceleration.y", "linear_acceleration.z",
               "linear_acceleration_covariance:0,0,0,0,0,0,0,0,0" }

    outImuData = { -1, -1, HEAD_FLAG }
    outImuData = tableConcat(outImuData, quat)
    outImuData = tableConcat(outImuData, { ARRAY_FLAG })
    outImuData = tableConcat(outImuData, angvel)
    outImuData = tableConcat(outImuData, { ARRAY_FLAG })
    outImuData = tableConcat(outImuData, accel)
    outImuData = tableConcat(outImuData, { ARRAY_FLAG })

    outObj = { '/sim/object_points', 'std_msgs/Float32MultiArray' }
    outObj[3] = tableToCommString(getObjectPoints())
    outObjData = { -1, -1, ARRAY_FLAG }

    outFloats = tableConcat(outDvlData, outPoseData)
    outFloats = tableConcat(outFloats, outObjData)
    outFloats = tableConcat(outFloats, outImuData)
    outStrings = tableConcat(outDvl, outPose)
    outStrings = tableConcat(outStrings, outObj)
    outStrings = tableConcat(outStrings, outImu)

    return {}, outFloats, outStrings, ""
end

function tableToCommString(tab)
    ret = "data:"
    for i = 1, #tab do
        ret = ret .. tab[i] .. ","
    end
    ret = string.sub(ret, 1, #ret - 1)
    print(ret)
    return ret
end

function getObjectPoints()
    seen = {}
    h = sim.getObjectAssociatedWithScript(sim.handle_self)
    seen[h] = true
    robotObjects = sim.getObjectsInTree(h)
    for i = 1, #robotObjects do
        seen[robotObjects[i]] = true
    end

    objects = {}

    allParents = sim.getObjectsInTree(sim.handle_scene)
    for k = 1, #allParents do
        children = sim.getObjectsInTree(allParents[k])
        if not seen[allParents[k]] then
            objects[sim.getObjectName(allParents[k])] = {}
        end
        for q = 1, #children do
            i = children[q]
            if not seen[i] then
                --print(sim.getObjectName(i).." belongs to "..sim.getObjectName(allParents[k]))
                seen[i] = true
                x = {}
                blah, x[1] = sim.getObjectFloatParameter(i, 15)
                blah, x[2] = sim.getObjectFloatParameter(i, 18)
                y = {}
                blah, y[1] = sim.getObjectFloatParameter(i, 16)
                blah, y[2] = sim.getObjectFloatParameter(i, 19)
                z = {}
                blah, z[1] = sim.getObjectFloatParameter(i, 17)
                blah, z[2] = sim.getObjectFloatParameter(i, 20)
                for j = 0, 7 do
                    xi = (j % 2) + 1
                    yi = (math.floor(j / 2) % 2) + 1
                    zi = (math.floor(j / 4) % 2) + 1
                    --print("" .. xi .. " " .. yi .. " " .. zi)
                    new_point = { x[xi], y[yi], z[zi] }
                    --print(new_point)
                    objects[sim.getObjectName(allParents[k])] = tableConcat(objects[sim.getObjectName(allParents[k])], new_point)
                    --print(#objects[sim.getObjectName(allParents[k])])
                end
            end
        end
    end

    ret = {}

    for key, value in pairs(objects) do
        --print(key)
        --print(value)
        ret = tableConcat(ret, { getObjectID(key) })
        ret = tableConcat(ret, value)
        ret = tableConcat(ret, { -999999 })
    end
    print(ret)

    return ret
end

function getObjectID(name)
    if string.find(string.lower(name), "gate") then
        return 1
    elseif string.find(string.lower(name), "buoy") then
        return 2
    end
    return -1
end