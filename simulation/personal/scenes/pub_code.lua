function tableConcat(t1,t2)
    for i=1,#t2 do
        t1[#t1+1] = t2[i]
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

    outDvl = {'/sim/dvl', 'geometry_msgs/TwistStamped', 'header', 
    	"twist.linear.x", "twist.linear.y", "twist.linear.z", 
		"twist.angular.x", "twist.angular.y", "twist.angular.z"}

	outDvlData = {-1, -1, HEAD_FLAG}
	outDvlData = tableConcat(outDvlData, linvel)
	outDvlData = tableConcat(outDvlData, angvel)

    outPose = {'/sim/pose', 'geometry_msgs/PoseStamped', 'header',
    	"pose.position.x", "pose.position.y", "pose.position.z",
    	"pose.orientation.x", "pose.orientation.y", "pose.orientation.z", "pose.orientation.w"}

    outPoseData = {-1, -1, HEAD_FLAG}
    outPoseData = tableConcat(outPoseData, pos)
    outPoseData = tableConcat(outPoseData, quat)

    outArray = {'/sim/test_array', 'std_msgs/Float32MultiArray', 'data:14,13,151,12'}
    outArrayData = {-1,-1,ARRAY_FLAG}

    outImu = {'/sim/imu', 'sensor_msgs/Imu', "header",  
    	"orientation.x", "orientation.y", "orientation.z", "orientation.w",
		"orientation_covariance:0,0,0,0,0,0,0,0,0",
		"angular_velocity.x", "angular_velocity.y", "angular_velocity.z",
		"angular_velocity_covariance:0,0,0,0,0,0,0,0,0",
		"linear_acceleration.x", "linear_acceleration.y", "linear_acceleration.z",
		"linear_acceleration_covariance:0,0,0,0,0,0,0,0,0"}

	outImuData = {-1, -1, HEAD_FLAG}
	outImuData = tableConcat(outImuData, quat)
	outImuData = tableConcat(outImuData, {ARRAY_FLAG})
	outImuData = tableConcat(outImuData, angvel)
	outImuData = tableConcat(outImuData, {ARRAY_FLAG})
	outImuData = tableConcat(outImuData, accel)
	outImuData = tableConcat(outImuData, {ARRAY_FLAG})

    outFloats = tableConcat(outDvlData, outPoseData)
    --outFloats = tableConcat(outFloats, outArrayData)
    outFloats = tableConcat(outFloats, outImuData)
    outStrings = tableConcat(outDvl, outPose)
    --outStrings = tableConcat(outStrings, outArray)
    outStrings = tableConcat(outStrings, outImu)

    return {}, outFloats, outStrings, ""
end

--[[
function ros_publishing(inInts, inFloats, inString, inBuffer)
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

    return {}, outFloats, {}, ""
end
]]--