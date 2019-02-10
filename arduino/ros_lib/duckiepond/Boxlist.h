#ifndef _ROS_duckiepond_Boxlist_h
#define _ROS_duckiepond_Boxlist_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "duckiepond/Box.h"
#include "sensor_msgs/CompressedImage.h"

namespace duckiepond
{

  class Boxlist : public ros::Msg
  {
    public:
      uint32_t list_length;
      typedef duckiepond::Box _list_type;
      _list_type st_list;
      _list_type * list;
      typedef int32_t _image_width_type;
      _image_width_type image_width;
      typedef int32_t _image_height_type;
      _image_height_type image_height;
      typedef sensor_msgs::CompressedImage _image_type;
      _image_type image;

    Boxlist():
      list_length(0), list(NULL),
      image_width(0),
      image_height(0),
      image()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->list_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->list_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->list_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->list_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->list_length);
      for( uint32_t i = 0; i < list_length; i++){
      offset += this->list[i].serialize(outbuffer + offset);
      }
      union {
        int32_t real;
        uint32_t base;
      } u_image_width;
      u_image_width.real = this->image_width;
      *(outbuffer + offset + 0) = (u_image_width.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_image_width.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_image_width.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_image_width.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->image_width);
      union {
        int32_t real;
        uint32_t base;
      } u_image_height;
      u_image_height.real = this->image_height;
      *(outbuffer + offset + 0) = (u_image_height.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_image_height.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_image_height.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_image_height.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->image_height);
      offset += this->image.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t list_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      list_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      list_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      list_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->list_length);
      if(list_lengthT > list_length)
        this->list = (duckiepond::Box*)realloc(this->list, list_lengthT * sizeof(duckiepond::Box));
      list_length = list_lengthT;
      for( uint32_t i = 0; i < list_length; i++){
      offset += this->st_list.deserialize(inbuffer + offset);
        memcpy( &(this->list[i]), &(this->st_list), sizeof(duckiepond::Box));
      }
      union {
        int32_t real;
        uint32_t base;
      } u_image_width;
      u_image_width.base = 0;
      u_image_width.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_image_width.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_image_width.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_image_width.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->image_width = u_image_width.real;
      offset += sizeof(this->image_width);
      union {
        int32_t real;
        uint32_t base;
      } u_image_height;
      u_image_height.base = 0;
      u_image_height.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_image_height.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_image_height.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_image_height.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->image_height = u_image_height.real;
      offset += sizeof(this->image_height);
      offset += this->image.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "duckiepond/Boxlist"; };
    const char * getMD5(){ return "f1c06fd753584eb32832fec4a435ca3b"; };

  };

}
#endif