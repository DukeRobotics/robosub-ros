#ifndef _ROS_data_pub_DVLRaw_h
#define _ROS_data_pub_DVLRaw_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace data_pub
{

  class DVLRaw : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef float _sa_roll_type;
      _sa_roll_type sa_roll;
      typedef float _sa_pitch_type;
      _sa_pitch_type sa_pitch;
      typedef float _sa_heading_type;
      _sa_heading_type sa_heading;
      typedef float _ts_salinity_type;
      _ts_salinity_type ts_salinity;
      typedef float _ts_temperature_type;
      _ts_temperature_type ts_temperature;
      typedef float _ts_depth_type;
      _ts_depth_type ts_depth;
      typedef float _ts_sound_speed_type;
      _ts_sound_speed_type ts_sound_speed;
      typedef int32_t _ts_built_in_test_type;
      _ts_built_in_test_type ts_built_in_test;
      typedef float _bi_x_axis_type;
      _bi_x_axis_type bi_x_axis;
      typedef float _bi_y_axis_type;
      _bi_y_axis_type bi_y_axis;
      typedef float _bi_z_axis_type;
      _bi_z_axis_type bi_z_axis;
      typedef float _bi_error_type;
      _bi_error_type bi_error;
      typedef const char* _bi_status_type;
      _bi_status_type bi_status;
      typedef float _bs_transverse_type;
      _bs_transverse_type bs_transverse;
      typedef float _bs_longitudinal_type;
      _bs_longitudinal_type bs_longitudinal;
      typedef float _bs_normal_type;
      _bs_normal_type bs_normal;
      typedef const char* _bs_status_type;
      _bs_status_type bs_status;
      typedef float _be_east_type;
      _be_east_type be_east;
      typedef float _be_north_type;
      _be_north_type be_north;
      typedef float _be_upwards_type;
      _be_upwards_type be_upwards;
      typedef const char* _be_status_type;
      _be_status_type be_status;
      typedef float _bd_east_type;
      _bd_east_type bd_east;
      typedef float _bd_north_type;
      _bd_north_type bd_north;
      typedef float _bd_upwards_type;
      _bd_upwards_type bd_upwards;
      typedef float _bd_range_type;
      _bd_range_type bd_range;
      typedef float _bd_time_type;
      _bd_time_type bd_time;

    DVLRaw():
      header(),
      sa_roll(0),
      sa_pitch(0),
      sa_heading(0),
      ts_salinity(0),
      ts_temperature(0),
      ts_depth(0),
      ts_sound_speed(0),
      ts_built_in_test(0),
      bi_x_axis(0),
      bi_y_axis(0),
      bi_z_axis(0),
      bi_error(0),
      bi_status(""),
      bs_transverse(0),
      bs_longitudinal(0),
      bs_normal(0),
      bs_status(""),
      be_east(0),
      be_north(0),
      be_upwards(0),
      be_status(""),
      bd_east(0),
      bd_north(0),
      bd_upwards(0),
      bd_range(0),
      bd_time(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->sa_roll);
      offset += serializeAvrFloat64(outbuffer + offset, this->sa_pitch);
      offset += serializeAvrFloat64(outbuffer + offset, this->sa_heading);
      offset += serializeAvrFloat64(outbuffer + offset, this->ts_salinity);
      offset += serializeAvrFloat64(outbuffer + offset, this->ts_temperature);
      offset += serializeAvrFloat64(outbuffer + offset, this->ts_depth);
      offset += serializeAvrFloat64(outbuffer + offset, this->ts_sound_speed);
      union {
        int32_t real;
        uint32_t base;
      } u_ts_built_in_test;
      u_ts_built_in_test.real = this->ts_built_in_test;
      *(outbuffer + offset + 0) = (u_ts_built_in_test.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ts_built_in_test.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ts_built_in_test.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ts_built_in_test.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ts_built_in_test);
      offset += serializeAvrFloat64(outbuffer + offset, this->bi_x_axis);
      offset += serializeAvrFloat64(outbuffer + offset, this->bi_y_axis);
      offset += serializeAvrFloat64(outbuffer + offset, this->bi_z_axis);
      offset += serializeAvrFloat64(outbuffer + offset, this->bi_error);
      uint32_t length_bi_status = strlen(this->bi_status);
      varToArr(outbuffer + offset, length_bi_status);
      offset += 4;
      memcpy(outbuffer + offset, this->bi_status, length_bi_status);
      offset += length_bi_status;
      offset += serializeAvrFloat64(outbuffer + offset, this->bs_transverse);
      offset += serializeAvrFloat64(outbuffer + offset, this->bs_longitudinal);
      offset += serializeAvrFloat64(outbuffer + offset, this->bs_normal);
      uint32_t length_bs_status = strlen(this->bs_status);
      varToArr(outbuffer + offset, length_bs_status);
      offset += 4;
      memcpy(outbuffer + offset, this->bs_status, length_bs_status);
      offset += length_bs_status;
      offset += serializeAvrFloat64(outbuffer + offset, this->be_east);
      offset += serializeAvrFloat64(outbuffer + offset, this->be_north);
      offset += serializeAvrFloat64(outbuffer + offset, this->be_upwards);
      uint32_t length_be_status = strlen(this->be_status);
      varToArr(outbuffer + offset, length_be_status);
      offset += 4;
      memcpy(outbuffer + offset, this->be_status, length_be_status);
      offset += length_be_status;
      offset += serializeAvrFloat64(outbuffer + offset, this->bd_east);
      offset += serializeAvrFloat64(outbuffer + offset, this->bd_north);
      offset += serializeAvrFloat64(outbuffer + offset, this->bd_upwards);
      offset += serializeAvrFloat64(outbuffer + offset, this->bd_range);
      offset += serializeAvrFloat64(outbuffer + offset, this->bd_time);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->sa_roll));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->sa_pitch));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->sa_heading));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->ts_salinity));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->ts_temperature));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->ts_depth));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->ts_sound_speed));
      union {
        int32_t real;
        uint32_t base;
      } u_ts_built_in_test;
      u_ts_built_in_test.base = 0;
      u_ts_built_in_test.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ts_built_in_test.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ts_built_in_test.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ts_built_in_test.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ts_built_in_test = u_ts_built_in_test.real;
      offset += sizeof(this->ts_built_in_test);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->bi_x_axis));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->bi_y_axis));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->bi_z_axis));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->bi_error));
      uint32_t length_bi_status;
      arrToVar(length_bi_status, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_bi_status; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_bi_status-1]=0;
      this->bi_status = (char *)(inbuffer + offset-1);
      offset += length_bi_status;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->bs_transverse));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->bs_longitudinal));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->bs_normal));
      uint32_t length_bs_status;
      arrToVar(length_bs_status, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_bs_status; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_bs_status-1]=0;
      this->bs_status = (char *)(inbuffer + offset-1);
      offset += length_bs_status;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->be_east));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->be_north));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->be_upwards));
      uint32_t length_be_status;
      arrToVar(length_be_status, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_be_status; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_be_status-1]=0;
      this->be_status = (char *)(inbuffer + offset-1);
      offset += length_be_status;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->bd_east));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->bd_north));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->bd_upwards));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->bd_range));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->bd_time));
     return offset;
    }

    const char * getType(){ return "data_pub/DVLRaw"; };
    const char * getMD5(){ return "dc52b68b3c2f39e5e3eed7a7af37a441"; };

  };

}
#endif