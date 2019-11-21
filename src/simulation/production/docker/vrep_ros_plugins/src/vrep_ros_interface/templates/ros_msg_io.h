#ifndef VREP_ROS_PLUGIN__ROS_MSG_IO__H
#define VREP_ROS_PLUGIN__ROS_MSG_IO__H

#include <ros_msg_builtin_io.h>
#include <vrep_ros_interface.h>

#py from parse_messages_and_services import get_msgs_info, get_srvs_info, get_msgs_srvs_info
#py msgs = get_msgs_info(pycpp.params['messages_file'])
#py srvs = get_srvs_info(pycpp.params['services_file'])
#py msgssrvs = get_msgs_srvs_info(pycpp.params['messages_file'], pycpp.params['services_file'])
#py for msg, info in msgs.items():
#include <`msg`.h>
#py endfor
#py for srv, info in srvs.items():
#include <`srv`.h>
#py endfor

#py for msg, info in msgssrvs.items():
void write__`info.typespec.normalized()`(const `info.typespec.ctype()`& msg, int stack, const WriteOptions *opt = NULL);
void read__`info.typespec.normalized()`(int stack, `info.typespec.ctype()` *msg, const ReadOptions *opt = NULL);
#py endfor
#py for msg, info in msgs.items():
void ros_callback__`info.typespec.normalized()`(const boost::shared_ptr<`info.typespec.ctype()` const>& msg, SubscriberProxy *proxy);
#py endfor

#endif // VREP_ROS_PLUGIN__ROS_MSG_IO__H
