#py from parse_messages_and_services import get_srvs_info
#py srvs = get_srvs_info(pycpp.params['services_file'])
#py for srv, info in srvs.items():
    else if(serviceClientProxy->serviceType == "`info.typespec.fullname`")
    {
        `info.typespec.ctype()` srv;
        read__`info.typespec.normalized()`Request(p->stackID, &(srv.request), &(serviceClientProxy->rd_opt));
        if(serviceClientProxy->client.call(srv))
        {
            write__`info.typespec.normalized()`Response(srv.response, p->stackID, &(serviceClientProxy->wr_opt));
        }
        else
        {
            throw exception("failed to call service `info.typespec.fullname`");
        }
    }
#py endfor
