#py from parse_messages_and_services import get_srvs_info
#py srvs = get_srvs_info(pycpp.params['services_file'])
#py for srv, info in srvs.items():
    else if(in->serviceType == "`info.typespec.fullname`")
    {
        serviceServerProxy->server = nh->advertiseService<`info.typespec.ctype()`::Request, `info.typespec.ctype()`::Response>(in->serviceName, boost::bind(ros_srv_callback__`info.typespec.normalized()`, _1, _2, serviceServerProxy));
    }
#py endfor
