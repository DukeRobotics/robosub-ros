#ifndef VREP_ROS_PLUGIN__ROS_SRV_IO__H
#define VREP_ROS_PLUGIN__ROS_SRV_IO__H

#include <ros_msg_builtin_io.h>
#include <ros/ros.h>
#include <vrep_ros_interface.h>

#py from parse_messages_and_services import get_srvs_info
#py srvs = get_srvs_info(pycpp.params['services_file'])
#py for srv, info in srvs.items():
#include <`srv`.h>
#py endfor

#py for srv, info in srvs.items():
bool ros_srv_callback__`info.typespec.normalized()`(`info.typespec.ctype()`::Request& req, `info.typespec.ctype()`::Response& res, ServiceServerProxy *proxy);
#py endfor

#endif // VREP_ROS_PLUGIN__ROS_SRV_IO__H
