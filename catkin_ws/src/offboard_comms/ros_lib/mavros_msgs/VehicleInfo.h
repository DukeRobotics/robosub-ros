#ifndef _ROS_mavros_msgs_VehicleInfo_h
#define _ROS_mavros_msgs_VehicleInfo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace mavros_msgs
{

  class VehicleInfo : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint8_t _available_info_type;
      _available_info_type available_info;
      typedef uint8_t _sysid_type;
      _sysid_type sysid;
      typedef uint8_t _compid_type;
      _compid_type compid;
      typedef uint8_t _autopilot_type;
      _autopilot_type autopilot;
      typedef uint8_t _type_type;
      _type_type type;
      typedef uint8_t _system_status_type;
      _system_status_type system_status;
      typedef uint8_t _base_mode_type;
      _base_mode_type base_mode;
      typedef uint32_t _custom_mode_type;
      _custom_mode_type custom_mode;
      typedef const char* _mode_type;
      _mode_type mode;
      typedef uint32_t _mode_id_type;
      _mode_id_type mode_id;
      typedef uint64_t _capabilities_type;
      _capabilities_type capabilities;
      typedef uint32_t _flight_sw_version_type;
      _flight_sw_version_type flight_sw_version;
      typedef uint32_t _middleware_sw_version_type;
      _middleware_sw_version_type middleware_sw_version;
      typedef uint32_t _os_sw_version_type;
      _os_sw_version_type os_sw_version;
      typedef uint32_t _board_version_type;
      _board_version_type board_version;
      typedef uint16_t _vendor_id_type;
      _vendor_id_type vendor_id;
      typedef uint16_t _product_id_type;
      _product_id_type product_id;
      typedef uint64_t _uid_type;
      _uid_type uid;
      enum { HAVE_INFO_HEARTBEAT =  1 };
      enum { HAVE_INFO_AUTOPILOT_VERSION =  2 };

    VehicleInfo():
      header(),
      available_info(0),
      sysid(0),
      compid(0),
      autopilot(0),
      type(0),
      system_status(0),
      base_mode(0),
      custom_mode(0),
      mode(""),
      mode_id(0),
      capabilities(0),
      flight_sw_version(0),
      middleware_sw_version(0),
      os_sw_version(0),
      board_version(0),
      vendor_id(0),
      product_id(0),
      uid(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->available_info >> (8 * 0)) & 0xFF;
      offset += sizeof(this->available_info);
      *(outbuffer + offset + 0) = (this->sysid >> (8 * 0)) & 0xFF;
      offset += sizeof(this->sysid);
      *(outbuffer + offset + 0) = (this->compid >> (8 * 0)) & 0xFF;
      offset += sizeof(this->compid);
      *(outbuffer + offset + 0) = (this->autopilot >> (8 * 0)) & 0xFF;
      offset += sizeof(this->autopilot);
      *(outbuffer + offset + 0) = (this->type >> (8 * 0)) & 0xFF;
      offset += sizeof(this->type);
      *(outbuffer + offset + 0) = (this->system_status >> (8 * 0)) & 0xFF;
      offset += sizeof(this->system_status);
      *(outbuffer + offset + 0) = (this->base_mode >> (8 * 0)) & 0xFF;
      offset += sizeof(this->base_mode);
      *(outbuffer + offset + 0) = (this->custom_mode >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->custom_mode >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->custom_mode >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->custom_mode >> (8 * 3)) & 0xFF;
      offset += sizeof(this->custom_mode);
      uint32_t length_mode = strlen(this->mode);
      varToArr(outbuffer + offset, length_mode);
      offset += 4;
      memcpy(outbuffer + offset, this->mode, length_mode);
      offset += length_mode;
      *(outbuffer + offset + 0) = (this->mode_id >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->mode_id >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->mode_id >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->mode_id >> (8 * 3)) & 0xFF;
      offset += sizeof(this->mode_id);
      union {
        uint64_t real;
        uint32_t base;
      } u_capabilities;
      u_capabilities.real = this->capabilities;
      *(outbuffer + offset + 0) = (u_capabilities.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_capabilities.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_capabilities.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_capabilities.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->capabilities);
      *(outbuffer + offset + 0) = (this->flight_sw_version >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->flight_sw_version >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->flight_sw_version >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->flight_sw_version >> (8 * 3)) & 0xFF;
      offset += sizeof(this->flight_sw_version);
      *(outbuffer + offset + 0) = (this->middleware_sw_version >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->middleware_sw_version >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->middleware_sw_version >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->middleware_sw_version >> (8 * 3)) & 0xFF;
      offset += sizeof(this->middleware_sw_version);
      *(outbuffer + offset + 0) = (this->os_sw_version >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->os_sw_version >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->os_sw_version >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->os_sw_version >> (8 * 3)) & 0xFF;
      offset += sizeof(this->os_sw_version);
      *(outbuffer + offset + 0) = (this->board_version >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->board_version >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->board_version >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->board_version >> (8 * 3)) & 0xFF;
      offset += sizeof(this->board_version);
      *(outbuffer + offset + 0) = (this->vendor_id >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->vendor_id >> (8 * 1)) & 0xFF;
      offset += sizeof(this->vendor_id);
      *(outbuffer + offset + 0) = (this->product_id >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->product_id >> (8 * 1)) & 0xFF;
      offset += sizeof(this->product_id);
      union {
        uint64_t real;
        uint32_t base;
      } u_uid;
      u_uid.real = this->uid;
      *(outbuffer + offset + 0) = (u_uid.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_uid.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_uid.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_uid.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->uid);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->available_info =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->available_info);
      this->sysid =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->sysid);
      this->compid =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->compid);
      this->autopilot =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->autopilot);
      this->type =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->type);
      this->system_status =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->system_status);
      this->base_mode =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->base_mode);
      this->custom_mode =  ((uint32_t) (*(inbuffer + offset)));
      this->custom_mode |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->custom_mode |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->custom_mode |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->custom_mode);
      uint32_t length_mode;
      arrToVar(length_mode, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_mode; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_mode-1]=0;
      this->mode = (char *)(inbuffer + offset-1);
      offset += length_mode;
      this->mode_id =  ((uint32_t) (*(inbuffer + offset)));
      this->mode_id |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->mode_id |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->mode_id |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->mode_id);
      union {
        uint64_t real;
        uint32_t base;
      } u_capabilities;
      u_capabilities.base = 0;
      u_capabilities.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_capabilities.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_capabilities.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_capabilities.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->capabilities = u_capabilities.real;
      offset += sizeof(this->capabilities);
      this->flight_sw_version =  ((uint32_t) (*(inbuffer + offset)));
      this->flight_sw_version |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->flight_sw_version |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->flight_sw_version |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->flight_sw_version);
      this->middleware_sw_version =  ((uint32_t) (*(inbuffer + offset)));
      this->middleware_sw_version |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->middleware_sw_version |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->middleware_sw_version |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->middleware_sw_version);
      this->os_sw_version =  ((uint32_t) (*(inbuffer + offset)));
      this->os_sw_version |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->os_sw_version |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->os_sw_version |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->os_sw_version);
      this->board_version =  ((uint32_t) (*(inbuffer + offset)));
      this->board_version |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->board_version |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->board_version |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->board_version);
      this->vendor_id =  ((uint16_t) (*(inbuffer + offset)));
      this->vendor_id |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->vendor_id);
      this->product_id =  ((uint16_t) (*(inbuffer + offset)));
      this->product_id |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->product_id);
      union {
        uint64_t real;
        uint32_t base;
      } u_uid;
      u_uid.base = 0;
      u_uid.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_uid.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_uid.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_uid.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->uid = u_uid.real;
      offset += sizeof(this->uid);
     return offset;
    }

    const char * getType(){ return "mavros_msgs/VehicleInfo"; };
    const char * getMD5(){ return "68ac9e63349db04d0cf8dd45a9a5b283"; };

  };

}
#endif