#ifndef _ROS_offboard_comms_ServoControl_h
#define _ROS_offboard_comms_ServoControl_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace offboard_comms
{

  class ServoControl : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint16_t speeds[8];

    ServoControl():
      header(),
      speeds()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      for( uint32_t i = 0; i < 8; i++){
      *(outbuffer + offset + 0) = (this->speeds[i] >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->speeds[i] >> (8 * 1)) & 0xFF;
      offset += sizeof(this->speeds[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      for( uint32_t i = 0; i < 8; i++){
      this->speeds[i] =  ((uint16_t) (*(inbuffer + offset)));
      this->speeds[i] |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->speeds[i]);
      }
     return offset;
    }

    const char * getType(){ return "offboard_comms/ServoControl"; };
    const char * getMD5(){ return "a3943b0c5b26f7205fbc76ab37e55454"; };

  };

}
#endif