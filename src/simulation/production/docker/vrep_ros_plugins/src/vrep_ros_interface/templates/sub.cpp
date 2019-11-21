#py from parse_messages_and_services import get_msgs_info
#py msgs = get_msgs_info(pycpp.params['messages_file'])
#py for msg, info in msgs.items():
    else if(in->topicType == "`info.typespec.fullname`")
    {
        subscriberProxy->subscriber = nh->subscribe<`info.typespec.ctype()`>(in->topicName, in->queueSize, boost::bind(ros_callback__`info.typespec.normalized()`, _1, subscriberProxy));
    }
#py endfor
