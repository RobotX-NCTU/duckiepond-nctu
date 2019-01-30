#ifndef _ROS_wireless_msgs_Connection_h
#define _ROS_wireless_msgs_Connection_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace wireless_msgs
{

  class Connection : public ros::Msg
  {
    public:
      typedef float _bitrate_type;
      _bitrate_type bitrate;
      typedef int16_t _txpower_type;
      _txpower_type txpower;
      typedef const char* _link_quality_raw_type;
      _link_quality_raw_type link_quality_raw;
      typedef float _link_quality_type;
      _link_quality_type link_quality;
      typedef int16_t _signal_level_type;
      _signal_level_type signal_level;
      typedef int16_t _noise_level_type;
      _noise_level_type noise_level;
      typedef const char* _essid_type;
      _essid_type essid;
      typedef const char* _bssid_type;
      _bssid_type bssid;
      typedef float _frequency_type;
      _frequency_type frequency;

    Connection():
      bitrate(0),
      txpower(0),
      link_quality_raw(""),
      link_quality(0),
      signal_level(0),
      noise_level(0),
      essid(""),
      bssid(""),
      frequency(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_bitrate;
      u_bitrate.real = this->bitrate;
      *(outbuffer + offset + 0) = (u_bitrate.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_bitrate.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_bitrate.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_bitrate.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->bitrate);
      union {
        int16_t real;
        uint16_t base;
      } u_txpower;
      u_txpower.real = this->txpower;
      *(outbuffer + offset + 0) = (u_txpower.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_txpower.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->txpower);
      uint32_t length_link_quality_raw = strlen(this->link_quality_raw);
      varToArr(outbuffer + offset, length_link_quality_raw);
      offset += 4;
      memcpy(outbuffer + offset, this->link_quality_raw, length_link_quality_raw);
      offset += length_link_quality_raw;
      union {
        float real;
        uint32_t base;
      } u_link_quality;
      u_link_quality.real = this->link_quality;
      *(outbuffer + offset + 0) = (u_link_quality.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_link_quality.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_link_quality.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_link_quality.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->link_quality);
      union {
        int16_t real;
        uint16_t base;
      } u_signal_level;
      u_signal_level.real = this->signal_level;
      *(outbuffer + offset + 0) = (u_signal_level.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_signal_level.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->signal_level);
      union {
        int16_t real;
        uint16_t base;
      } u_noise_level;
      u_noise_level.real = this->noise_level;
      *(outbuffer + offset + 0) = (u_noise_level.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_noise_level.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->noise_level);
      uint32_t length_essid = strlen(this->essid);
      varToArr(outbuffer + offset, length_essid);
      offset += 4;
      memcpy(outbuffer + offset, this->essid, length_essid);
      offset += length_essid;
      uint32_t length_bssid = strlen(this->bssid);
      varToArr(outbuffer + offset, length_bssid);
      offset += 4;
      memcpy(outbuffer + offset, this->bssid, length_bssid);
      offset += length_bssid;
      offset += serializeAvrFloat64(outbuffer + offset, this->frequency);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_bitrate;
      u_bitrate.base = 0;
      u_bitrate.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_bitrate.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_bitrate.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_bitrate.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->bitrate = u_bitrate.real;
      offset += sizeof(this->bitrate);
      union {
        int16_t real;
        uint16_t base;
      } u_txpower;
      u_txpower.base = 0;
      u_txpower.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_txpower.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->txpower = u_txpower.real;
      offset += sizeof(this->txpower);
      uint32_t length_link_quality_raw;
      arrToVar(length_link_quality_raw, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_link_quality_raw; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_link_quality_raw-1]=0;
      this->link_quality_raw = (char *)(inbuffer + offset-1);
      offset += length_link_quality_raw;
      union {
        float real;
        uint32_t base;
      } u_link_quality;
      u_link_quality.base = 0;
      u_link_quality.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_link_quality.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_link_quality.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_link_quality.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->link_quality = u_link_quality.real;
      offset += sizeof(this->link_quality);
      union {
        int16_t real;
        uint16_t base;
      } u_signal_level;
      u_signal_level.base = 0;
      u_signal_level.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_signal_level.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->signal_level = u_signal_level.real;
      offset += sizeof(this->signal_level);
      union {
        int16_t real;
        uint16_t base;
      } u_noise_level;
      u_noise_level.base = 0;
      u_noise_level.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_noise_level.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->noise_level = u_noise_level.real;
      offset += sizeof(this->noise_level);
      uint32_t length_essid;
      arrToVar(length_essid, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_essid; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_essid-1]=0;
      this->essid = (char *)(inbuffer + offset-1);
      offset += length_essid;
      uint32_t length_bssid;
      arrToVar(length_bssid, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_bssid; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_bssid-1]=0;
      this->bssid = (char *)(inbuffer + offset-1);
      offset += length_bssid;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->frequency));
     return offset;
    }

    const char * getType(){ return "wireless_msgs/Connection"; };
    const char * getMD5(){ return "f1a0d060cec0bda21949621526aa056e"; };

  };

}
#endif