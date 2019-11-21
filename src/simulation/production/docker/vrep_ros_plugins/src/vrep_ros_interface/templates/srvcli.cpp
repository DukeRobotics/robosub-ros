#py from parse_messages_and_services import get_srvs_info
#py srvs = get_srvs_info(pycpp.params['services_file'])
#py for srv, info in srvs.items():
    else if(in->serviceType == "`info.typespec.fullname`")
    {
        serviceClientProxy->client = nh->serviceClient<`info.typespec.ctype()`>(in->serviceName);
    }
#py endfor
