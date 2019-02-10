#ifndef _ROS_phidgets_high_speed_encoder_EncoderDecimatedSpeed_h
#define _ROS_phidgets_high_speed_encoder_EncoderDecimatedSpeed_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace phidgets_high_speed_encoder
{

  class EncoderDecimatedSpeed : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef float _avr_speed_type;
      _avr_speed_type avr_speed;

    EncoderDecimatedSpeed():
      header(),
      avr_speed(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->avr_speed);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->avr_speed));
     return offset;
    }

    const char * getType(){ return "phidgets_high_speed_encoder/EncoderDecimatedSpeed"; };
    const char * getMD5(){ return "20fbdbe041b6e052c8c414d50464f125"; };

  };

}
#endif