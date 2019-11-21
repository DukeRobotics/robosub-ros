#py from parse_messages_and_services import get_msgs_info
#py msgs = get_msgs_info(pycpp.params['messages_file'])
#py for msg, info in msgs.items():
    else if(publisherProxy->topicType == "`info.typespec.fullname`")
    {
        `info.typespec.ctype()` msg;
        read__`info.typespec.normalized()`(p->stackID, &msg, &(publisherProxy->rd_opt));
        publisherProxy->publisher.publish(msg);
    }
#py endfor
