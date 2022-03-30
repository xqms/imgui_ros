// Topic selector combobox
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#ifndef IMGUI_ROS_TOPIC_SELECTOR_H
#define IMGUI_ROS_TOPIC_SELECTOR_H

#include <string>
#include <set>
#include <vector>

namespace imgui_ros
{

class TopicSelector
{
public:
    explicit TopicSelector(const std::initializer_list<std::string>& types) noexcept
     : TopicSelector(std::set<std::string>{types})
    {}

    explicit TopicSelector(const std::set<std::string>& types) noexcept;
    ~TopicSelector() noexcept;

    bool draw(const char* label, std::string* selectedTopic, std::string* selectedType = nullptr);
private:
    void updateTopics();

    struct TopicInfo
    {
        std::string topic;
        std::string type;
    };

    std::vector<TopicInfo> m_topics;
    std::set<std::string> m_types;
};

}

#endif
