// Handy class that gives access to the last message on a topic
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#ifndef IMGUI_ROS_BOX_H
#define IMGUI_ROS_BOX_H

#include <ros/node_handle.h>

namespace imgui_ros
{

template<class Msg>
class Box
{
public:
    Box()
    {}

    explicit Box(ros::NodeHandle& nh, const std::string& topic, const ros::TransportHints& hints = {})
     : m_nh{nh}
     , m_topic{topic}
     , m_hints{hints}
    {
        m_sub = m_nh.subscribe(topic, 1, &Box<Msg>::handleData, this, hints);
    }

    Box(const Box& other)
     : Box(other.m_nh, other.m_topic, other.m_hints)
    {
    }

    Box& operator=(const Box& other)
    {
        m_nh = other.m_nh;
        m_topic = other.m_topic;
        m_hints = other.m_hints;

        if(!m_topic.empty())
            m_sub = m_nh.subscribe(m_topic, 1, &Box<Msg>::handleData, this, m_hints);
        else
            m_sub = {};

        return *this;
    }

    boost::shared_ptr<const Msg> message() const
    { return m_msg; }

    operator bool() const
    { return !!m_msg; }

    const Msg* operator->() const
    { return m_msg.get(); }

private:
    void handleData(const boost::shared_ptr<const Msg>& msg)
    {
        m_msg = msg;
    }

    ros::NodeHandle m_nh;
    std::string m_topic;
    ros::TransportHints m_hints;
    ros::Subscriber m_sub;
    boost::shared_ptr<const Msg> m_msg;
};

}

#endif
