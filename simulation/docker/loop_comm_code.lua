rospubs = {}

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

function get_ros_data(inInts, inFloats, inString, inBuffer)
	i = 1
	while i <= #inString do
		if strsub(inString[i],1,1)== '/' then
			rospubs[#rospubs+1] = simROS.advertise(inStrings[i], inStrings[i+1])
    		simROS.publisherTreatUInt8ArrayAsString(rospubs[#rospubs])
    		i += 2
		end
		temp = rospubs[#rospubs]
		for path in mysplit(inString[i], '.') do
			if temp[path] == nil then
				temp[path] = {}
			end
			temp = temp[path]
		end
		temp = inInts[i]
		i += 1
	end
end