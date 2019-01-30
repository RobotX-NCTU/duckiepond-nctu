#ifndef _ROS_wireless_msgs_Quality_h
#define _ROS_wireless_msgs_Quality_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace wireless_msgs
{

  class Quality : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint16_t _messages_received_type;
      _messages_received_type messages_received;
      typedef uint16_t _messages_missed_type;
      _messages_missed_type messages_missed;
      typedef uint32_t _total_length_type;
      _total_length_type total_length;
      uint32_t message_lengths_length;
      typedef uint32_t _message_lengths_type;
      _message_lengths_type st_message_lengths;
      _message_lengths_type * message_lengths;
      typedef float _latency_avg_type;
      _latency_avg_type latency_avg;
      uint32_t latency_measurements_length;
      typedef float _latency_measurements_type;
      _latency_measurements_type st_latency_measurements;
      _latency_measurements_type * latency_measurements;

    Quality():
      header(),
      messages_received(0),
      messages_missed(0),
      total_length(0),
      message_lengths_length(0), message_lengths(NULL),
      latency_avg(0),
      latency_measurements_length(0), latency_measurements(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->messages_received >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->messages_received >> (8 * 1)) & 0xFF;
      offset += sizeof(this->messages_received);
      *(outbuffer + offset + 0) = (this->messages_missed >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->messages_missed >> (8 * 1)) & 0xFF;
      offset += sizeof(this->messages_missed);
      *(outbuffer + offset + 0) = (this->total_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->total_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->total_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->total_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->total_length);
      *(outbuffer + offset + 0) = (this->message_lengths_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->message_lengths_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->message_lengths_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->message_lengths_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->message_lengths_length);
      for( uint32_t i = 0; i < message_lengths_length; i++){
      *(outbuffer + offset + 0) = (this->message_lengths[i] >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->message_lengths[i] >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->message_lengths[i] >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->message_lengths[i] >> (8 * 3)) & 0xFF;
      offset += sizeof(this->message_lengths[i]);
      }
      union {
        float real;
        uint32_t base;
      } u_latency_avg;
      u_latency_avg.real = this->latency_avg;
      *(outbuffer + offset + 0) = (u_latency_avg.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_latency_avg.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_latency_avg.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_latency_avg.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->latency_avg);
      *(outbuffer + offset + 0) = (this->latency_measurements_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->latency_measurements_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->latency_measurements_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->latency_measurements_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->latency_measurements_length);
      for( uint32_t i = 0; i < latency_measurements_length; i++){
      union {
        float real;
        uint32_t base;
      } u_latency_measurementsi;
      u_latency_measurementsi.real = this->latency_measurements[i];
      *(outbuffer + offset + 0) = (u_latency_measurementsi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_latency_measurementsi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_latency_measurementsi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_latency_measurementsi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->latency_measurements[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->messages_received =  ((uint16_t) (*(inbuffer + offset)));
      this->messages_received |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->messages_received);
      this->messages_missed =  ((uint16_t) (*(inbuffer + offset)));
      this->messages_missed |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->messages_missed);
      this->total_length =  ((uint32_t) (*(inbuffer + offset)));
      this->total_length |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->total_length |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->total_length |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->total_length);
      uint32_t message_lengths_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      message_lengths_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      message_lengths_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      message_lengths_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->message_lengths_length);
      if(message_lengths_lengthT > message_lengths_length)
        this->message_lengths = (uint32_t*)realloc(this->message_lengths, message_lengths_lengthT * sizeof(uint32_t));
      message_lengths_length = message_lengths_lengthT;
      for( uint32_t i = 0; i < message_lengths_length; i++){
      this->st_message_lengths =  ((uint32_t) (*(inbuffer + offset)));
      this->st_message_lengths |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->st_message_lengths |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->st_message_lengths |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->st_message_lengths);
        memcpy( &(this->message_lengths[i]), &(this->st_message_lengths), sizeof(uint32_t));
      }
      union {
        float real;
        uint32_t base;
      } u_latency_avg;
      u_latency_avg.base = 0;
      u_latency_avg.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_latency_avg.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_latency_avg.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_latency_avg.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->latency_avg = u_latency_avg.real;
      offset += sizeof(this->latency_avg);
      uint32_t latency_measurements_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      latency_measurements_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      latency_measurements_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      latency_measurements_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->latency_measurements_length);
      if(latency_measurements_lengthT > latency_measurements_length)
        this->latency_measurements = (float*)realloc(this->latency_measurements, latency_measurements_lengthT * sizeof(float));
      latency_measurements_length = latency_measurements_lengthT;
      for( uint32_t i = 0; i < latency_measurements_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_latency_measurements;
      u_st_latency_measurements.base = 0;
      u_st_latency_measurements.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_latency_measurements.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_latency_measurements.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_latency_measurements.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_latency_measurements = u_st_latency_measurements.real;
      offset += sizeof(this->st_latency_measurements);
        memcpy( &(this->latency_measurements[i]), &(this->st_latency_measurements), sizeof(float));
      }
     return offset;
    }

    const char * getType(){ return "wireless_msgs/Quality"; };
    const char * getMD5(){ return "5362dac71ff196cfe73d1dc68d4c58ed"; };

  };

}
#endif