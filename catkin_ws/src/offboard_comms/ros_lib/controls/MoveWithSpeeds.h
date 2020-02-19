#ifndef _ROS_controls_MoveWithSpeeds_h
#define _ROS_controls_MoveWithSpeeds_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace controls
{

  class MoveWithSpeeds : public ros::Msg
  {
    public:
      float speeds[6];

    MoveWithSpeeds():
      speeds()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      for( uint32_t i = 0; i < 6; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->speeds[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      for( uint32_t i = 0; i < 6; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->speeds[i]));
      }
     return offset;
    }

    const char * getType(){ return "controls/MoveWithSpeeds"; };
    const char * getMD5(){ return "c186147ff06599aeaabd68c92f43a02b"; };

  };

}
#endif