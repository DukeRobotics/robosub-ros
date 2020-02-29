#ifndef _ROS_offboard_comms_ThrusterSpeeds_h
#define _ROS_offboard_comms_ThrusterSpeeds_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace offboard_comms
{

  class ThrusterSpeeds : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      int8_t speeds[8];

    ThrusterSpeeds():
      header(),
      speeds()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      for( uint32_t i = 0; i < 8; i++){
      union {
        int8_t real;
        uint8_t base;
      } u_speedsi;
      u_speedsi.real = this->speeds[i];
      *(outbuffer + offset + 0) = (u_speedsi.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->speeds[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      for( uint32_t i = 0; i < 8; i++){
      union {
        int8_t real;
        uint8_t base;
      } u_speedsi;
      u_speedsi.base = 0;
      u_speedsi.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->speeds[i] = u_speedsi.real;
      offset += sizeof(this->speeds[i]);
      }
     return offset;
    }

    const char * getType(){ return "offboard_comms/ThrusterSpeeds"; };
    const char * getMD5(){ return "162db394a586ad904b91af2ff77226fa"; };

  };

}
#endif