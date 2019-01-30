#ifndef _ROS_wireless_msgs_Network_h
#define _ROS_wireless_msgs_Network_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace wireless_msgs
{

  class Network : public ros::Msg
  {
    public:
      typedef const char* _type_type;
      _type_type type;
      typedef const char* _essid_type;
      _essid_type essid;
      typedef const char* _mac_type;
      _mac_type mac;
      typedef const char* _mode_type;
      _mode_type mode;
      typedef const char* _frequency_type;
      _frequency_type frequency;
      typedef bool _encryption_type;
      _encryption_type encryption;

    Network():
      type(""),
      essid(""),
      mac(""),
      mode(""),
      frequency(""),
      encryption(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_type = strlen(this->type);
      varToArr(outbuffer + offset, length_type);
      offset += 4;
      memcpy(outbuffer + offset, this->type, length_type);
      offset += length_type;
      uint32_t length_essid = strlen(this->essid);
      varToArr(outbuffer + offset, length_essid);
      offset += 4;
      memcpy(outbuffer + offset, this->essid, length_essid);
      offset += length_essid;
      uint32_t length_mac = strlen(this->mac);
      varToArr(outbuffer + offset, length_mac);
      offset += 4;
      memcpy(outbuffer + offset, this->mac, length_mac);
      offset += length_mac;
      uint32_t length_mode = strlen(this->mode);
      varToArr(outbuffer + offset, length_mode);
      offset += 4;
      memcpy(outbuffer + offset, this->mode, length_mode);
      offset += length_mode;
      uint32_t length_frequency = strlen(this->frequency);
      varToArr(outbuffer + offset, length_frequency);
      offset += 4;
      memcpy(outbuffer + offset, this->frequency, length_frequency);
      offset += length_frequency;
      union {
        bool real;
        uint8_t base;
      } u_encryption;
      u_encryption.real = this->encryption;
      *(outbuffer + offset + 0) = (u_encryption.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->encryption);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_type;
      arrToVar(length_type, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_type; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_type-1]=0;
      this->type = (char *)(inbuffer + offset-1);
      offset += length_type;
      uint32_t length_essid;
      arrToVar(length_essid, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_essid; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_essid-1]=0;
      this->essid = (char *)(inbuffer + offset-1);
      offset += length_essid;
      uint32_t length_mac;
      arrToVar(length_mac, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_mac; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_mac-1]=0;
      this->mac = (char *)(inbuffer + offset-1);
      offset += length_mac;
      uint32_t length_mode;
      arrToVar(length_mode, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_mode; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_mode-1]=0;
      this->mode = (char *)(inbuffer + offset-1);
      offset += length_mode;
      uint32_t length_frequency;
      arrToVar(length_frequency, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_frequency; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_frequency-1]=0;
      this->frequency = (char *)(inbuffer + offset-1);
      offset += length_frequency;
      union {
        bool real;
        uint8_t base;
      } u_encryption;
      u_encryption.base = 0;
      u_encryption.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->encryption = u_encryption.real;
      offset += sizeof(this->encryption);
     return offset;
    }

    const char * getType(){ return "wireless_msgs/Network"; };
    const char * getMD5(){ return "24b3a97e0e3dc13fb92e036725f91d11"; };

  };

}
#endif