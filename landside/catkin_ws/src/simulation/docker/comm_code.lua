require "math"

function extsysCall_init()
    h=sim.getObjectAssociatedWithScript(sim.handle_self)
    m=sim.getShapeMassAndInertia(h)
    simRemoteApi.start(8080)

    moveSub=simROS.subscribe('/sim/move', 'std_msgs/Float32MultiArray', 'move_callback')
    simROS.subscriberTreatUInt8ArrayAsString(moveSub)

    HEAD_FLAG = -525600
    ARRAY_FLAG = -8675309
    rospubs = {}
    last = nil
    seq = 0;
end

function extsysCall_actuation()

end

function move_callback(msg)
    last = msg.data
    print(last)
end

function mysplit (inputstr, sep)
    if sep == nil then
            sep = "%s"
    end
    local t={}
    for str in string.gmatch(inputstr, "([^"..sep.."]+)") do
            table.insert(t, str)
    end
    return t
end

function dump(o)
   if type(o) == 'table' then
      local s = '{ '
      for k,v in pairs(o) do
         if type(k) ~= 'number' then k = '"'..k..'"' end
         s = s .. '['..k..'] = ' .. dump(v) .. ','
      end
      return s .. '} '
   else
      return tostring(o)
   end
end

function get_ros_data(inInts, inFloats, inStrings, inBuffer)
	if #inStrings >= 2 then
		head = {}
	    head["seq"] = seq
	    seq = seq + 1
	    head["stamp"] = simROS.getTime()
	    head["frame_id"] = "global"
	    i = 1
	    lastpub = nil
		while i <= #inStrings do
			if string.sub(inStrings[i],1,1)== '/' then
				if lastpub ~= nil then
					print(dump(pub))
					simROS.publish(lastpub, pub)
				end
				if rospubs[inStrings[i]] == nil then
					rospubs[inStrings[i]] = simROS.advertise(inStrings[i], inStrings[i+1])
		    		simROS.publisherTreatUInt8ArrayAsString(rospubs[inStrings[i]])
		    	end
		    	lastpub = rospubs[inStrings[i]]
		    	pub = {}
	    		i = i + 2
			end
			--temp = rospubs[inStrings[i-2]]
			temp = pub
			tobesplit = inStrings[i]
			if inFloats[i] == ARRAY_FLAG then
				tobesplit = mysplit(tobesplit, ':')
				arrayString = tobesplit[2]
				tobesplit = tobesplit[1]
			end
			splitstr = mysplit(tobesplit, '.')
			for k = 1,#splitstr do
				path = splitstr[k]
				if temp[path] == nil then
					temp[path] = {}
				end
				if path ~= splitstr[#splitstr] then
					temp = temp[path]
				end
				lastpath = path
			end
			temp[lastpath] = inFloats[i]
			if temp[lastpath] == HEAD_FLAG then
				temp[lastpath] = head
			end
			if temp[lastpath] == ARRAY_FLAG then
				print(dump(arrayFromString(arrayString)))
				temp[lastpath] = arrayFromString(arrayString)
			end
			i = i + 1
		end
		if (lastpub ~= nil) then
			print(dump(pub))
			simROS.publish(lastpub, pub)
		end
	end
	if last == nil then
        return {}, {0,0,0,0,0,0,0,0}, {}, ''
    end
    return {}, last, {}, ''
end

function arrayFromString( arrStr )
	ret = {}	nums = mysplit(arrStr, ',')
	for i = 1,#nums do
		ret[i] = tonumber(nums[i])
	end
	return ret
end
