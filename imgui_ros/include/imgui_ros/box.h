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

    explicit Box(const rclcpp::Node::SharedPtr& node, const std::string& topic, const rclcpp::QoS& qos = rclcpp::QoS{1})
     : m_node{node}
     , m_topic{topic}
     , m_qos{qos}
    {
        subscribe();
    }

    Box(const Box& other)
     : Box(other.m_node, other.m_topic, other.m_qos)
    {
    }

    Box& operator=(const Box& other)
    {
        m_node = other.m_node;
        m_topic = other.m_topic;
        m_qos = other.m_qos;

        subscribe();

        return *this;
    }

    std::shared_ptr<const Msg> message() const
    { return m_msg; }

    operator bool() const
    { return !!m_msg; }

    const Msg* operator->() const
    { return m_msg.get(); }

private:
    void subscribe()
    {
        if(m_node && !m_topic.empty())
            m_sub = m_node->template create_subscription<Msg>(m_topic, m_qos, [this](const std::shared_ptr<const Msg>& msg){ handleData(msg); });
        else
            m_sub = {};
    }

    void handleData(const std::shared_ptr<const Msg>& msg)
    {
        m_msg = msg;
    }

    rclcpp::Node::SharedPtr m_node;
    std::string m_topic;
    rclcpp::QoS m_qos{1};
    typename rclcpp::Subscription<Msg>::SharedPtr m_sub;
    std::shared_ptr<const Msg> m_msg;
};

}

#endif
