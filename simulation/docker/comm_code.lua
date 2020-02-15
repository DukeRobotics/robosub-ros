require "math"

function extsysCall_init()
    h=sim.getObjectAssociatedWithScript(sim.handle_self)
    m=sim.getShapeMassAndInertia(h)
    simRemoteApi.start(8080)
    moveSub=simROS.subscribe('/sim/move', 'std_msgs/Float32MultiArray', 'move_callback')
    last = nil
    seq = 0;
    
    simROS.subscriberTreatUInt8ArrayAsString(moveSub)
    dvlPub=simROS.advertise('/sim/dvl', 'geometry_msgs/TwistStamped')
    simROS.publisherTreatUInt8ArrayAsString(dvlPub)
    posePub=simROS.advertise('/sim/pose', 'geometry_msgs/PoseStamped')
    simROS.publisherTreatUInt8ArrayAsString(dvlPub)
    imuPub=simROS.advertise('/sim/imu', 'sensor_msgs/Imu')
    simROS.publisherTreatUInt8ArrayAsString(imuPub)
    

    --test, should get removed
    --testdata = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}
    --get_ros_data({}, testdata, {}, '')
end

function extsysCall_actuation() 
    --get_ros_data({}, testdata, {}, '')
end

function move_callback(msg)
    last = msg.data
    print(last)
end

function get_ros_data(inInts, inFloats, inString, inBuffer)
    if inString[1]=="stop" then last = nil end
    -- 3 linvel, 3 angvel, 4 quat, 3 accel, 3 pos, 1 time
    if #inFloats>=16 then
        head = {}
        head["seq"] = seq
        seq = seq + 1
        head["stamp"] = simROS.getTime()
        head["frame_id"] = "global"
        linear = {}
        angular = {}
        quat = {}
        accel = {}
        pos = {}
        linear['x'] = inFloats[1]
        linear['y'] = inFloats[2]
        linear['z'] = inFloats[3]
        angular['x'] = inFloats[4]
        angular['y'] = inFloats[5]
        angular['z'] = inFloats[6]
        quat['x'] = inFloats[7]
        quat['y'] = inFloats[8]
        quat['z'] = inFloats[9]
        quat['w'] = inFloats[10]
        accel['x'] = inFloats[11]
        accel['y'] = inFloats[12]
        accel['z'] = inFloats[13]
        pos['x'] = inFloats[14]
        pos['y'] = inFloats[15]
        pos['z'] = inFloats[16]
        
        cov = {0,0,0,0,0,0,0,0,0} --all zeros because unknown
        
        imu = {}
        imu['header'] = head
        imu['orientation'] = quat
        imu['orientation_covariance'] = cov
        imu['angular_velocity'] = angular
        imu['angular_velocity_covariance'] = cov
        imu['linear_acceleration'] = accel
        imu['linear_acceleration_covariance'] = cov

        poseS = {}
        pose = {}
        pose['position'] = pos
        pose['orientation'] = quat
        poseS['header'] = head
        poseS['pose'] = pose
        

        twistS = {}
        twist = {}
        twist['linear'] = linear
        twist['angular'] = angular
        twistS['header'] = head
        twistS['twist'] = twist

        simROS.publish(dvlPub, twistS)
        simROS.publish(posePub, poseS)
        simROS.publish(imuPub, imu)
        
        
    end
    if last == nil then
        return {}, {0,0,0,0,0,0,0,0}, {}, ''
    end
    return {}, last, {}, ""
end




