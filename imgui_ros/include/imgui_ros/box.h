// Handy class that gives access to the last message on a topic
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#ifndef IMGUI_ROS_BOX_H
#define IMGUI_ROS_BOX_H

#include <rclcpp/node.hpp>

namespace imgui_ros
{

template<class Msg>
class Box
{
public:
    Box()
    {}

    explicit Box(const rclcpp::Node::SharedPtr& nh, const std::string& topic, const rclcpp::QoS& hints = {})
     : m_nh{nh}
     , m_topic{topic}
     , m_hints{hints}
    {
        m_sub = m_nh->create_subscription<Msg>(topic, hints, [&](const std::shared_ptr<Msg>& msg){ this->handleData(msg); });
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
            m_sub = m_nh->create_subscription(m_topic, m_hints, [&](const std::shared_ptr<Msg>& msg){ this->handleData(msg); });
        else
            m_sub = {};

        return *this;
    }

    std::shared_ptr<const Msg> message() const
    { return m_msg; }

    operator bool() const
    { return !!m_msg; }

    const Msg* operator->() const
    { return m_msg.get(); }

private:
    void handleData(const std::shared_ptr<const Msg>& msg)
    {
        m_msg = msg;
    }

    rclcpp::Node::SharedPtr m_nh;
    std::string m_topic;
    rclcpp::QoS m_hints;
    typename rclcpp::Subscription<Msg>::SharedPtr m_sub;
    std::shared_ptr<const Msg> m_msg;
};

}

#endif
