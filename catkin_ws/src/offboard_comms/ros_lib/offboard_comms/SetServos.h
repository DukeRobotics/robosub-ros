#ifndef _ROS_SERVICE_SetServos_h
#define _ROS_SERVICE_SetServos_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace offboard_comms
{

static const char SETSERVOS[] = "offboard_comms/SetServos";

  class SetServosRequest : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint16_t angles[8];

    SetServosRequest():
      header(),
      angles()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      for( uint32_t i = 0; i < 8; i++){
      *(outbuffer + offset + 0) = (this->angles[i] >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->angles[i] >> (8 * 1)) & 0xFF;
      offset += sizeof(this->angles[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      for( uint32_t i = 0; i < 8; i++){
      this->angles[i] =  ((uint16_t) (*(inbuffer + offset)));
      this->angles[i] |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->angles[i]);
      }
     return offset;
    }

    const char * getType(){ return SETSERVOS; };
    const char * getMD5(){ return "d8daa224c1eb5523cacf95cadd64bfa9"; };

  };

  class SetServosResponse : public ros::Msg
  {
    public:

    SetServosResponse()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
     return offset;
    }

    const char * getType(){ return SETSERVOS; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class SetServos {
    public:
    typedef SetServosRequest Request;
    typedef SetServosResponse Response;
  };

}
#endif
