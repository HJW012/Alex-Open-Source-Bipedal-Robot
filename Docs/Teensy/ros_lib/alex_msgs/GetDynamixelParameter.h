#ifndef _ROS_alex_msgs_GetDynamixelParameter_h
#define _ROS_alex_msgs_GetDynamixelParameter_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace alex_msgs
{

  class GetDynamixelParameter : public ros::Msg
  {
    public:
      typedef uint8_t _id_type;
      _id_type id;
      typedef uint16_t _parameter_type;
      _parameter_type parameter;

    GetDynamixelParameter():
      id(0),
      parameter(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->id >> (8 * 0)) & 0xFF;
      offset += sizeof(this->id);
      *(outbuffer + offset + 0) = (this->parameter >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->parameter >> (8 * 1)) & 0xFF;
      offset += sizeof(this->parameter);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->id =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->id);
      this->parameter =  ((uint16_t) (*(inbuffer + offset)));
      this->parameter |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->parameter);
     return offset;
    }

    virtual const char * getType() override { return "alex_msgs/GetDynamixelParameter"; };
    virtual const char * getMD5() override { return "09529ee40bf8247788aa121dbaf491a4"; };

  };

}
#endif
