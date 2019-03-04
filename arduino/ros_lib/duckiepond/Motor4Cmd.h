#ifndef _ROS_duckiepond_Motor4Cmd_h
#define _ROS_duckiepond_Motor4Cmd_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace duckiepond
{

  class Motor4Cmd : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef float _lf_type;
      _lf_type lf;
      typedef float _rf_type;
      _rf_type rf;
      typedef float _lr_type;
      _lr_type lr;
      typedef float _rr_type;
      _rr_type rr;

    Motor4Cmd():
      header(),
      lf(0),
      rf(0),
      lr(0),
      rr(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_lf;
      u_lf.real = this->lf;
      *(outbuffer + offset + 0) = (u_lf.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_lf.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_lf.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_lf.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->lf);
      union {
        float real;
        uint32_t base;
      } u_rf;
      u_rf.real = this->rf;
      *(outbuffer + offset + 0) = (u_rf.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rf.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rf.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rf.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rf);
      union {
        float real;
        uint32_t base;
      } u_lr;
      u_lr.real = this->lr;
      *(outbuffer + offset + 0) = (u_lr.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_lr.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_lr.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_lr.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->lr);
      union {
        float real;
        uint32_t base;
      } u_rr;
      u_rr.real = this->rr;
      *(outbuffer + offset + 0) = (u_rr.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rr.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rr.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rr.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rr);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_lf;
      u_lf.base = 0;
      u_lf.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_lf.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_lf.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_lf.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->lf = u_lf.real;
      offset += sizeof(this->lf);
      union {
        float real;
        uint32_t base;
      } u_rf;
      u_rf.base = 0;
      u_rf.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rf.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_rf.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_rf.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->rf = u_rf.real;
      offset += sizeof(this->rf);
      union {
        float real;
        uint32_t base;
      } u_lr;
      u_lr.base = 0;
      u_lr.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_lr.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_lr.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_lr.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->lr = u_lr.real;
      offset += sizeof(this->lr);
      union {
        float real;
        uint32_t base;
      } u_rr;
      u_rr.base = 0;
      u_rr.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rr.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_rr.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_rr.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->rr = u_rr.real;
      offset += sizeof(this->rr);
     return offset;
    }

    const char * getType(){ return "duckiepond/Motor4Cmd"; };
    const char * getMD5(){ return "77d80b96b13055bc97b62acface81cb7"; };

  };

}
#endif