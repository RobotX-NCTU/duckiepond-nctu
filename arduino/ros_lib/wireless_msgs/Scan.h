#ifndef _ROS_wireless_msgs_Scan_h
#define _ROS_wireless_msgs_Scan_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "wireless_msgs/Network.h"

namespace wireless_msgs
{

  class Scan : public ros::Msg
  {
    public:
      uint32_t networks_length;
      typedef wireless_msgs::Network _networks_type;
      _networks_type st_networks;
      _networks_type * networks;

    Scan():
      networks_length(0), networks(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->networks_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->networks_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->networks_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->networks_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->networks_length);
      for( uint32_t i = 0; i < networks_length; i++){
      offset += this->networks[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t networks_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      networks_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      networks_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      networks_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->networks_length);
      if(networks_lengthT > networks_length)
        this->networks = (wireless_msgs::Network*)realloc(this->networks, networks_lengthT * sizeof(wireless_msgs::Network));
      networks_length = networks_lengthT;
      for( uint32_t i = 0; i < networks_length; i++){
      offset += this->st_networks.deserialize(inbuffer + offset);
        memcpy( &(this->networks[i]), &(this->st_networks), sizeof(wireless_msgs::Network));
      }
     return offset;
    }

    const char * getType(){ return "wireless_msgs/Scan"; };
    const char * getMD5(){ return "d32911a086af571ae6871a223a6112f0"; };

  };

}
#endif