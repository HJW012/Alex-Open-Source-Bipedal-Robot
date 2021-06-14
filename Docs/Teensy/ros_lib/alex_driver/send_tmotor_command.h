#ifndef _ROS_SERVICE_send_tmotor_command_h
#define _ROS_SERVICE_send_tmotor_command_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "alex_msgs/MotorParamOut.h"

namespace alex_driver
{

static const char SEND_TMOTOR_COMMAND[] = "alex_driver/send_tmotor_command";

  class send_tmotor_commandRequest : public ros::Msg
  {
    public:
      typedef alex_msgs::MotorParamOut _motorParamOut_type;
      _motorParamOut_type motorParamOut;

    send_tmotor_commandRequest():
      motorParamOut()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->motorParamOut.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->motorParamOut.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return SEND_TMOTOR_COMMAND; };
    virtual const char * getMD5() override { return "2300c498babe9970590227b8a5c22946"; };

  };

  class send_tmotor_commandResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;

    send_tmotor_commandResponse():
      success(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.real = this->success;
      *(outbuffer + offset + 0) = (u_success.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.base = 0;
      u_success.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->success = u_success.real;
      offset += sizeof(this->success);
     return offset;
    }

    virtual const char * getType() override { return SEND_TMOTOR_COMMAND; };
    virtual const char * getMD5() override { return "358e233cde0c8a8bcfea4ce193f8fc15"; };

  };

  class send_tmotor_command {
    public:
    typedef send_tmotor_commandRequest Request;
    typedef send_tmotor_commandResponse Response;
  };

}
#endif
