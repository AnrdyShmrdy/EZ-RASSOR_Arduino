#ifndef _ROS_SERVICE_GetRoverStatus_h
#define _ROS_SERVICE_GetRoverStatus_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Pose.h"

namespace ezrassor_swarm_control
{

static const char GETROVERSTATUS[] = "ezrassor_swarm_control/GetRoverStatus";

  class GetRoverStatusRequest : public ros::Msg
  {
    public:

    GetRoverStatusRequest()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
     return offset;
    }

    const char * getType(){ return GETROVERSTATUS; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class GetRoverStatusResponse : public ros::Msg
  {
    public:
      typedef geometry_msgs::Pose _pose_type;
      _pose_type pose;
      typedef int8_t _battery_type;
      _battery_type battery;

    GetRoverStatusResponse():
      pose(),
      battery(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->pose.serialize(outbuffer + offset);
      union {
        int8_t real;
        uint8_t base;
      } u_battery;
      u_battery.real = this->battery;
      *(outbuffer + offset + 0) = (u_battery.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->battery);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->pose.deserialize(inbuffer + offset);
      union {
        int8_t real;
        uint8_t base;
      } u_battery;
      u_battery.base = 0;
      u_battery.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->battery = u_battery.real;
      offset += sizeof(this->battery);
     return offset;
    }

    const char * getType(){ return GETROVERSTATUS; };
    const char * getMD5(){ return "23a42361826957f3308fcb8f0f183f3b"; };

  };

  class GetRoverStatus {
    public:
    typedef GetRoverStatusRequest Request;
    typedef GetRoverStatusResponse Response;
  };

}
#endif
