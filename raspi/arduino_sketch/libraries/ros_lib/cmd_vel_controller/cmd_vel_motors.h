#ifndef _ROS_cmd_vel_controller_cmd_vel_motors_h
#define _ROS_cmd_vel_controller_cmd_vel_motors_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace cmd_vel_controller
{

  class cmd_vel_motors : public ros::Msg
  {
    public:
      typedef float _vel_1_type;
      _vel_1_type vel_1;
      typedef float _vel_2_type;
      _vel_2_type vel_2;
      typedef float _vel_3_type;
      _vel_3_type vel_3;
      typedef float _vel_4_type;
      _vel_4_type vel_4;

    cmd_vel_motors():
      vel_1(0),
      vel_2(0),
      vel_3(0),
      vel_4(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_vel_1;
      u_vel_1.real = this->vel_1;
      *(outbuffer + offset + 0) = (u_vel_1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_vel_1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_vel_1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_vel_1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->vel_1);
      union {
        float real;
        uint32_t base;
      } u_vel_2;
      u_vel_2.real = this->vel_2;
      *(outbuffer + offset + 0) = (u_vel_2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_vel_2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_vel_2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_vel_2.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->vel_2);
      union {
        float real;
        uint32_t base;
      } u_vel_3;
      u_vel_3.real = this->vel_3;
      *(outbuffer + offset + 0) = (u_vel_3.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_vel_3.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_vel_3.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_vel_3.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->vel_3);
      union {
        float real;
        uint32_t base;
      } u_vel_4;
      u_vel_4.real = this->vel_4;
      *(outbuffer + offset + 0) = (u_vel_4.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_vel_4.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_vel_4.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_vel_4.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->vel_4);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_vel_1;
      u_vel_1.base = 0;
      u_vel_1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_vel_1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_vel_1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_vel_1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->vel_1 = u_vel_1.real;
      offset += sizeof(this->vel_1);
      union {
        float real;
        uint32_t base;
      } u_vel_2;
      u_vel_2.base = 0;
      u_vel_2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_vel_2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_vel_2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_vel_2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->vel_2 = u_vel_2.real;
      offset += sizeof(this->vel_2);
      union {
        float real;
        uint32_t base;
      } u_vel_3;
      u_vel_3.base = 0;
      u_vel_3.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_vel_3.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_vel_3.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_vel_3.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->vel_3 = u_vel_3.real;
      offset += sizeof(this->vel_3);
      union {
        float real;
        uint32_t base;
      } u_vel_4;
      u_vel_4.base = 0;
      u_vel_4.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_vel_4.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_vel_4.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_vel_4.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->vel_4 = u_vel_4.real;
      offset += sizeof(this->vel_4);
     return offset;
    }

    const char * getType(){ return "cmd_vel_controller/cmd_vel_motors"; };
    const char * getMD5(){ return "ea47411d0a2d7f75bddda9caa1a9f2f0"; };

  };

}
#endif