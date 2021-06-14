#ifndef _ROS_SERVICE_send_can_h
#define _ROS_SERVICE_send_can_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Bool.h"
#include "can_msgs/Frame.h"

namespace alex_driver
{

static const char SEND_CAN[] = "alex_driver/send_can";

  class send_canRequest : public ros::Msg
  {
    public:
      typedef can_msgs::Frame _msg_type;
      _msg_type msg;

    send_canRequest():
      msg()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->msg.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->msg.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return SEND_CAN; };
    virtual const char * getMD5() override { return "dba6a08f3bff45fe6c83bcb1f3522ec2"; };

  };

  class send_canResponse : public ros::Msg
  {
    public:
      typedef std_msgs::Bool _success_type;
      _success_type success;

    send_canResponse():
      success()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->success.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->success.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return SEND_CAN; };
    virtual const char * getMD5() override { return "5f31cb2e813cfb0e488c574c3b5d9dbe"; };

  };

  class send_can {
    public:
    typedef send_canRequest Request;
    typedef send_canResponse Response;
  };

}
#endif
