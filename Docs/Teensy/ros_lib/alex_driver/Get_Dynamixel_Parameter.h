#ifndef _ROS_SERVICE_Get_Dynamixel_Parameter_h
#define _ROS_SERVICE_Get_Dynamixel_Parameter_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace alex_driver
{

static const char GET_DYNAMIXEL_PARAMETER[] = "alex_driver/Get_Dynamixel_Parameter";

  class Get_Dynamixel_ParameterRequest : public ros::Msg
  {
    public:
      typedef uint8_t _id_type;
      _id_type id;
      typedef uint16_t _parameter_type;
      _parameter_type parameter;

    Get_Dynamixel_ParameterRequest():
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

    virtual const char * getType() override { return GET_DYNAMIXEL_PARAMETER; };
    virtual const char * getMD5() override { return "09529ee40bf8247788aa121dbaf491a4"; };

  };

  class Get_Dynamixel_ParameterResponse : public ros::Msg
  {
    public:
      typedef int32_t _value_type;
      _value_type value;

    Get_Dynamixel_ParameterResponse():
      value(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_value;
      u_value.real = this->value;
      *(outbuffer + offset + 0) = (u_value.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_value.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_value.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_value.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->value);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_value;
      u_value.base = 0;
      u_value.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_value.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_value.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_value.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->value = u_value.real;
      offset += sizeof(this->value);
     return offset;
    }

    virtual const char * getType() override { return GET_DYNAMIXEL_PARAMETER; };
    virtual const char * getMD5() override { return "b3087778e93fcd34cc8d65bc54e850d1"; };

  };

  class Get_Dynamixel_Parameter {
    public:
    typedef Get_Dynamixel_ParameterRequest Request;
    typedef Get_Dynamixel_ParameterResponse Response;
  };

}
#endif
