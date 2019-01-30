#ifndef _ROS_SERVICE_Reset_h
#define _ROS_SERVICE_Reset_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace um6
{

static const char RESET[] = "um6/Reset";

  class ResetRequest : public ros::Msg
  {
    public:
      typedef bool _zero_gyros_type;
      _zero_gyros_type zero_gyros;
      typedef bool _reset_ekf_type;
      _reset_ekf_type reset_ekf;
      typedef bool _set_mag_ref_type;
      _set_mag_ref_type set_mag_ref;
      typedef bool _set_accel_ref_type;
      _set_accel_ref_type set_accel_ref;

    ResetRequest():
      zero_gyros(0),
      reset_ekf(0),
      set_mag_ref(0),
      set_accel_ref(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_zero_gyros;
      u_zero_gyros.real = this->zero_gyros;
      *(outbuffer + offset + 0) = (u_zero_gyros.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->zero_gyros);
      union {
        bool real;
        uint8_t base;
      } u_reset_ekf;
      u_reset_ekf.real = this->reset_ekf;
      *(outbuffer + offset + 0) = (u_reset_ekf.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->reset_ekf);
      union {
        bool real;
        uint8_t base;
      } u_set_mag_ref;
      u_set_mag_ref.real = this->set_mag_ref;
      *(outbuffer + offset + 0) = (u_set_mag_ref.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->set_mag_ref);
      union {
        bool real;
        uint8_t base;
      } u_set_accel_ref;
      u_set_accel_ref.real = this->set_accel_ref;
      *(outbuffer + offset + 0) = (u_set_accel_ref.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->set_accel_ref);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_zero_gyros;
      u_zero_gyros.base = 0;
      u_zero_gyros.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->zero_gyros = u_zero_gyros.real;
      offset += sizeof(this->zero_gyros);
      union {
        bool real;
        uint8_t base;
      } u_reset_ekf;
      u_reset_ekf.base = 0;
      u_reset_ekf.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->reset_ekf = u_reset_ekf.real;
      offset += sizeof(this->reset_ekf);
      union {
        bool real;
        uint8_t base;
      } u_set_mag_ref;
      u_set_mag_ref.base = 0;
      u_set_mag_ref.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->set_mag_ref = u_set_mag_ref.real;
      offset += sizeof(this->set_mag_ref);
      union {
        bool real;
        uint8_t base;
      } u_set_accel_ref;
      u_set_accel_ref.base = 0;
      u_set_accel_ref.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->set_accel_ref = u_set_accel_ref.real;
      offset += sizeof(this->set_accel_ref);
     return offset;
    }

    const char * getType(){ return RESET; };
    const char * getMD5(){ return "88aed859f96ffbc3fdce434a0b135bb3"; };

  };

  class ResetResponse : public ros::Msg
  {
    public:

    ResetResponse()
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

    const char * getType(){ return RESET; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class Reset {
    public:
    typedef ResetRequest Request;
    typedef ResetResponse Response;
  };

}
#endif
