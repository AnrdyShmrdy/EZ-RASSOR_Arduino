#ifndef _ROS_SERVICE_PreemptPath_h
#define _ROS_SERVICE_PreemptPath_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ezrassor_swarm_control
{

static const char PREEMPTPATH[] = "ezrassor_swarm_control/PreemptPath";

  class PreemptPathRequest : public ros::Msg
  {
    public:

    PreemptPathRequest()
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

    const char * getType(){ return PREEMPTPATH; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class PreemptPathResponse : public ros::Msg
  {
    public:

    PreemptPathResponse()
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

    const char * getType(){ return PREEMPTPATH; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class PreemptPath {
    public:
    typedef PreemptPathRequest Request;
    typedef PreemptPathResponse Response;
  };

}
#endif
