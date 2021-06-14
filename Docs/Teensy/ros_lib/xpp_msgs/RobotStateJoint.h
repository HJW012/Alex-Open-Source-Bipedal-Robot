#ifndef _ROS_xpp_msgs_RobotStateJoint_h
#define _ROS_xpp_msgs_RobotStateJoint_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/duration.h"
#include "xpp_msgs/State6d.h"
#include "sensor_msgs/JointState.h"

namespace xpp_msgs
{

  class RobotStateJoint : public ros::Msg
  {
    public:
      typedef ros::Duration _time_from_start_type;
      _time_from_start_type time_from_start;
      typedef xpp_msgs::State6d _base_type;
      _base_type base;
      typedef sensor_msgs::JointState _joint_state_type;
      _joint_state_type joint_state;
      uint32_t ee_contact_length;
      typedef bool _ee_contact_type;
      _ee_contact_type st_ee_contact;
      _ee_contact_type * ee_contact;

    RobotStateJoint():
      time_from_start(),
      base(),
      joint_state(),
      ee_contact_length(0), st_ee_contact(), ee_contact(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->time_from_start.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->time_from_start.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->time_from_start.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->time_from_start.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->time_from_start.sec);
      *(outbuffer + offset + 0) = (this->time_from_start.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->time_from_start.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->time_from_start.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->time_from_start.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->time_from_start.nsec);
      offset += this->base.serialize(outbuffer + offset);
      offset += this->joint_state.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->ee_contact_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->ee_contact_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->ee_contact_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->ee_contact_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ee_contact_length);
      for( uint32_t i = 0; i < ee_contact_length; i++){
      union {
        bool real;
        uint8_t base;
      } u_ee_contacti;
      u_ee_contacti.real = this->ee_contact[i];
      *(outbuffer + offset + 0) = (u_ee_contacti.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->ee_contact[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->time_from_start.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->time_from_start.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->time_from_start.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->time_from_start.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->time_from_start.sec);
      this->time_from_start.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->time_from_start.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->time_from_start.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->time_from_start.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->time_from_start.nsec);
      offset += this->base.deserialize(inbuffer + offset);
      offset += this->joint_state.deserialize(inbuffer + offset);
      uint32_t ee_contact_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      ee_contact_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      ee_contact_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      ee_contact_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->ee_contact_length);
      if(ee_contact_lengthT > ee_contact_length)
        this->ee_contact = (bool*)realloc(this->ee_contact, ee_contact_lengthT * sizeof(bool));
      ee_contact_length = ee_contact_lengthT;
      for( uint32_t i = 0; i < ee_contact_length; i++){
      union {
        bool real;
        uint8_t base;
      } u_st_ee_contact;
      u_st_ee_contact.base = 0;
      u_st_ee_contact.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->st_ee_contact = u_st_ee_contact.real;
      offset += sizeof(this->st_ee_contact);
        memcpy( &(this->ee_contact[i]), &(this->st_ee_contact), sizeof(bool));
      }
     return offset;
    }

    virtual const char * getType() override { return "xpp_msgs/RobotStateJoint"; };
    virtual const char * getMD5() override { return "e95013ef0a2eb2815e5e1a85c92e0ac0"; };

  };

}
#endif
