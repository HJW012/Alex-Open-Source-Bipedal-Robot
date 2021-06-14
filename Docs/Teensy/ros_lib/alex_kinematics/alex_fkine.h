#ifndef _ROS_SERVICE_alex_fkine_h
#define _ROS_SERVICE_alex_fkine_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/TransformStamped.h"

namespace alex_kinematics
{

static const char ALEX_FKINE[] = "alex_kinematics/alex_fkine";

  class alex_fkineRequest : public ros::Msg
  {
    public:
      uint32_t transforms_length;
      typedef geometry_msgs::TransformStamped _transforms_type;
      _transforms_type st_transforms;
      _transforms_type * transforms;

    alex_fkineRequest():
      transforms_length(0), st_transforms(), transforms(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->transforms_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->transforms_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->transforms_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->transforms_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->transforms_length);
      for( uint32_t i = 0; i < transforms_length; i++){
      offset += this->transforms[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t transforms_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      transforms_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      transforms_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      transforms_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->transforms_length);
      if(transforms_lengthT > transforms_length)
        this->transforms = (geometry_msgs::TransformStamped*)realloc(this->transforms, transforms_lengthT * sizeof(geometry_msgs::TransformStamped));
      transforms_length = transforms_lengthT;
      for( uint32_t i = 0; i < transforms_length; i++){
      offset += this->st_transforms.deserialize(inbuffer + offset);
        memcpy( &(this->transforms[i]), &(this->st_transforms), sizeof(geometry_msgs::TransformStamped));
      }
     return offset;
    }

    virtual const char * getType() override { return ALEX_FKINE; };
    virtual const char * getMD5() override { return "94810edda583a504dfda3829e70d7eec"; };

  };

  class alex_fkineResponse : public ros::Msg
  {
    public:
      uint32_t transforms_length;
      typedef geometry_msgs::TransformStamped _transforms_type;
      _transforms_type st_transforms;
      _transforms_type * transforms;

    alex_fkineResponse():
      transforms_length(0), st_transforms(), transforms(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->transforms_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->transforms_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->transforms_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->transforms_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->transforms_length);
      for( uint32_t i = 0; i < transforms_length; i++){
      offset += this->transforms[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t transforms_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      transforms_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      transforms_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      transforms_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->transforms_length);
      if(transforms_lengthT > transforms_length)
        this->transforms = (geometry_msgs::TransformStamped*)realloc(this->transforms, transforms_lengthT * sizeof(geometry_msgs::TransformStamped));
      transforms_length = transforms_lengthT;
      for( uint32_t i = 0; i < transforms_length; i++){
      offset += this->st_transforms.deserialize(inbuffer + offset);
        memcpy( &(this->transforms[i]), &(this->st_transforms), sizeof(geometry_msgs::TransformStamped));
      }
     return offset;
    }

    virtual const char * getType() override { return ALEX_FKINE; };
    virtual const char * getMD5() override { return "94810edda583a504dfda3829e70d7eec"; };

  };

  class alex_fkine {
    public:
    typedef alex_fkineRequest Request;
    typedef alex_fkineResponse Response;
  };

}
#endif
