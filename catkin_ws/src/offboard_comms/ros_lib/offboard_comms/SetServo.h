#ifndef _ROS_SERVICE_SetServo_h
#define _ROS_SERVICE_SetServo_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace offboard_comms
{

static const char SETSERVO[] = "offboard_comms/SetServo";

  class SetServoRequest : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint8_t _num_type;
      _num_type num;
      typedef uint16_t _angle_type;
      _angle_type angle;

    SetServoRequest():
      header(),
      num(0),
      angle(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->num >> (8 * 0)) & 0xFF;
      offset += sizeof(this->num);
      *(outbuffer + offset + 0) = (this->angle >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->angle >> (8 * 1)) & 0xFF;
      offset += sizeof(this->angle);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->num =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->num);
      this->angle =  ((uint16_t) (*(inbuffer + offset)));
      this->angle |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->angle);
     return offset;
    }

    const char * getType(){ return SETSERVO; };
    const char * getMD5(){ return "909d4f0a94fd1db0dbd25d8a7782333f"; };

  };

  class SetServoResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;

    SetServoResponse():
      success(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.real = this->success;
      *(outbuffer + offset + 0) = (u_success.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.base = 0;
      u_success.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->success = u_success.real;
      offset += sizeof(this->success);
     return offset;
    }

    const char * getType(){ return SETSERVO; };
    const char * getMD5(){ return "358e233cde0c8a8bcfea4ce193f8fc15"; };

  };

  class SetServo {
    public:
    typedef SetServoRequest Request;
    typedef SetServoResponse Response;
  };

}
#endif
