// Topic selector combobox
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#ifndef IMGUI_ROS_TOPIC_SELECTOR_H
#define IMGUI_ROS_TOPIC_SELECTOR_H

#include <string>
#include <set>
#include <vector>

#include <rclcpp/node.hpp>

namespace imgui_ros
{

class TopicSelector
{
public:
    explicit TopicSelector(const rclcpp::Node::SharedPtr& node, const std::initializer_list<std::string>& types) noexcept
     : TopicSelector(node, std::set<std::string>{types})
    {}

    explicit TopicSelector(const rclcpp::Node::SharedPtr& node, const std::set<std::string>& types) noexcept;
    ~TopicSelector() noexcept;

    bool draw(const char* label, std::string* selectedTopic, std::string* selectedType = nullptr);
private:
    void updateTopics();

    struct TopicInfo
    {
        std::string topic;
        std::string type;

        bool operator<(const TopicInfo& other) const
        {
            return topic < other.topic;
        }
    };

    rclcpp::Node::SharedPtr m_node;
    std::vector<TopicInfo> m_topics;
    std::set<std::string> m_types;
};

}

#endif
