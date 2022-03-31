// Provides abstraction of the outer GUI framework
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#ifndef IMGUI_ROS_CONTEXT_H
#define IMGUI_ROS_CONTEXT_H

#include <ros/subscriber.h>
#include <ros/transport_hints.h>

#include "box.h"

namespace imgui_ros
{

class Context
{
public:
    virtual void setWindowTitle(const std::string& title) = 0;
    virtual void setUpdateRate(float updateRate) = 0;

    virtual ros::NodeHandle& nodeHandle() = 0;

    template<class Msg>
    Box<Msg> subscribeBox(const std::string& topic, const ros::TransportHints& hints = {})
    { return Box<Msg>(nodeHandle(), topic, hints); }
};

}

#endif
