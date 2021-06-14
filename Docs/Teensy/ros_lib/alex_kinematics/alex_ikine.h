#ifndef _ROS_SERVICE_alex_ikine_h
#define _ROS_SERVICE_alex_ikine_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/TransformStamped.h"

namespace alex_kinematics
{

static const char ALEX_IKINE[] = "alex_kinematics/alex_ikine";

  class alex_ikineRequest : public ros::Msg
  {
    public:
      uint32_t footTransforms_length;
      typedef geometry_msgs::TransformStamped _footTransforms_type;
      _footTransforms_type st_footTransforms;
      _footTransforms_type * footTransforms;

    alex_ikineRequest():
      footTransforms_length(0), st_footTransforms(), footTransforms(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->footTransforms_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->footTransforms_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->footTransforms_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->footTransforms_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->footTransforms_length);
      for( uint32_t i = 0; i < footTransforms_length; i++){
      offset += this->footTransforms[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t footTransforms_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      footTransforms_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      footTransforms_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      footTransforms_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->footTransforms_length);
      if(footTransforms_lengthT > footTransforms_length)
        this->footTransforms = (geometry_msgs::TransformStamped*)realloc(this->footTransforms, footTransforms_lengthT * sizeof(geometry_msgs::TransformStamped));
      footTransforms_length = footTransforms_lengthT;
      for( uint32_t i = 0; i < footTransforms_length; i++){
      offset += this->st_footTransforms.deserialize(inbuffer + offset);
        memcpy( &(this->footTransforms[i]), &(this->st_footTransforms), sizeof(geometry_msgs::TransformStamped));
      }
     return offset;
    }

    virtual const char * getType() override { return ALEX_IKINE; };
    virtual const char * getMD5() override { return "8351a2c908fd9c611240b28ec7332047"; };

  };

  class alex_ikineResponse : public ros::Msg
  {
    public:
      typedef sensor_msgs::JointState _jointStates_type;
      _jointStates_type jointStates;

    alex_ikineResponse():
      jointStates()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->jointStates.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->jointStates.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return ALEX_IKINE; };
    virtual const char * getMD5() override { return "ce0f83a232331f23d6f38258c641fa10"; };

  };

  class alex_ikine {
    public:
    typedef alex_ikineRequest Request;
    typedef alex_ikineResponse Response;
  };

}
#endif
